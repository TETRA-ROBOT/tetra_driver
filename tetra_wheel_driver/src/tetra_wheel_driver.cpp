#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// Custom Service
#include "interfaces/srv/parameter_read.hpp"
#include "interfaces/srv/parameter_write.hpp"
#include "interfaces/srv/set_move_mode.hpp"
#include "interfaces/srv/linear_position_move.hpp"
#include "interfaces/srv/angular_position_move.hpp"
#include "interfaces/srv/servo.hpp"
// 버전 응답 필드(보드 종류·날짜·버전)가 같아 전원 보드용 타입을 함께 쓴다
#include "interfaces/srv/power_version_read.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
	#include "drive_module.h"
	#include "dssp_rs232_drive_module.h"
#ifdef __cplusplus
}
#endif

#include <chrono>
#include <thread>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <bitset>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <string>
#include <algorithm>

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

// Dynamic Physical Parameters
double m_dWheel_Radius = 0.1033;   // m
double m_dWheel_Distance = 0.4330; // m
double m_dTread_Width = 0.04;      // m

// serial
int com_port = 0;
char port[64] = {0,};

// calc
bool first = true;
double prev_coordinates[3] = {0.0, };
double coordinates[3] = {0.0, }; // x, y, theta
double dt = 0.0;
double velocity[3] = {0.0, };

// cmd_vel
double input_linear = 0.0, input_angular = 0.0;
double control_linear = 0.0, control_angular = 0.0;
double linear = 0.0, angular = 0.0, bt_linear = 0.0, bt_angular = 0.0;
int m_old_accel_data = 50;

// Pose
double m_dX_pos = 0.0;
double m_dY_pos = 0.0;
double m_dTheta = 0.0;
int Reset = 0;

// Velocity
double m_dVelocity_l = 0.0;
double m_dVelocity_r = 0.0;

// bumper & emg
int m_bumper_data = 0;
int m_emg_state = 0;
bool m_bflag_bumper = false;

// Error Code
int m_left_error_code = 0;
int m_right_error_code = 0;

// 에러 정보를 담을 구조체
struct MotorError {
	std::string contents;
	std::string cause;
	std::string measures;
};

// 에러 코드(int)를 받아 해당하는 에러 정보를 반환하는 함수
MotorError GetMotorErrorInfo(int error_code) {
	switch(error_code) {
		case 48: // 0x30 ('0')
			return {"Normal state", "-", "-"};
			
		case 49: // 0x31 ('1')
			return {"Emergency Stop", 
					"1) Emergency stop switch pressed", 
					"Release emergency stop switch"};
					
		case 50: // 0x32 ('2')
			return {"Motor hall sensor error", 
					"1) Hall sensor receiver error\n2) Motor line disconnected", 
					"Check motor / Check motor line"};
					
		case 51: // 0x33 ('3')
			return {"Encoder error", 
					"1) Encoder line error\n2) Motor line disconnected or miswired\n3) Driver gain setting error\n4) Power module damaged\n5) Encoder receiver error", 
					"Check encoder wiring\nCheck motor line system\nGain tuning"};
					
		case 52: // 0x34 ('4')
			return {"Detect Over Voltage", 
					"1) Power voltage exceeded rating\n2) Power module damaged\n3) Accel/decel setting error", 
					"Check power voltage\nCheck driver board\nChange parameter settings"};
					
		case 53: // 0x35 ('5')
			return {"Detect Under Voltage", 
					"1) Power voltage below rating", 
					"Check power voltage"};
					
		case 54: // 0x36 ('6')
			return {"Detect Over Load", 
					"1) Motor rated torque exceeds driver max output\n2) Torque limit setting error\n3) Operating speed/accel/decel setting error\n4) Obstacle in operating area", 
					"Check motor rated torque\nChange torque limit setting\nChange speed/accel/decel setting\nRemove obstacle"};
					
		case 55: // 0x37 ('7')
			return {"Detect Over Speed", 
					"1) Speed command exceeds rated speed", 
					"Change driver max speed setting"};
					
		case 56: // 0x38 ('8')
			return {"Detect Following Error", 
					"1) Following parameter setting error\n2) Driver gain setting error", 
					"Change parameter settings\nCheck encoder/motor line"};
					
		default:
			return {"Unknown Error", 
					"Undefined error code", 
					"Check manual"};
	}
}

// 에러 로그를 포맷팅하여 출력하는 함수
void PrintMotorErrorLog(const char* side, int error_code) {
	if (error_code == 48) return; // 정상 상태는 무시

	MotorError err = GetMotorErrorInfo(error_code);
	
	printf("\n==================================================\n");
	printf("[Motor Driver Error - %s Wheel]\n", side);
	printf(" - Code     : 0x%X ('%c')\n", error_code, error_code);
	printf(" - Contents : %s\n", err.contents.c_str());
	printf(" - Cause    : %s\n", err.cause.c_str());
	printf(" - Measures : %s\n", err.measures.c_str());
	printf("==================================================\n\n");
}

constexpr int BUMPER_FRONT_MASK = 0x01;
constexpr int SERVO_MASK = 0x02;
constexpr int BUMPER_BACK_MASK = 0x08;

bool IsServoEnabled(int data)
{
	return (data & SERVO_MASK) == 0;
}

bool IsFrontBumperActive(int data)
{
	return (data & BUMPER_FRONT_MASK) != 0;
}

bool IsBackBumperActive(int data)
{
	return (data & BUMPER_BACK_MASK) != 0;
}

void PrintBumperServoState(int data)
{
	std::bitset<4> bits(static_cast<unsigned long>(data));
	printf("[bumper/servo] %s -> servo:%s front:%s back:%s\n",
		bits.to_string().c_str(),
		IsServoEnabled(data) ? "ON" : "OFF",
		IsFrontBumperActive(data) ? "ON" : "OFF",
		IsBackBumperActive(data) ? "ON" : "OFF");
}

// Position move
int m_iPOS_Y = 0;
int m_iPOS_Theta = 0;
bool bPosition_mode_flag = false;

// emg one time check flag
bool m_bCheck_emg = true;

// Joystick value
double joy_linear = 1.0;
double joy_angular = 1.0;

// Parameter data
int m_iParam = 0;

// ekf_localization
bool m_bEKF_option = false;
bool m_bForwardCheck = false;

// Joystick Enable & Disable
bool m_bFlag_joy_enable = false;

// Flags to control callback execution
bool m_bpending_ = true;
std::mutex mutex_;

std::atomic<bool> stop_requested(false);

// 시그널 핸들러 안에서는 printf 같은 함수를 쓰면 안 되므로 플래그만 세운다. 종료 로그는 루프를 빠져나간 뒤 남긴다
void signal_handler(int signal) 
{
    (void)signal;
    stop_requested = true;
}

// 모터 드라이버 에러 코드: '0'(48) 정상, '1'(49) 비상정지, '5'(53) 저전압
constexpr int kMotorErrorNone = 48;
constexpr int kMotorErrorEmergencyStop = 49;
constexpr int kMotorErrorUnderVoltage = 53;

class TETRAWheelDriver: public rclcpp::Node
{
public:
	TETRAWheelDriver() : Node("tetra_wheel_driver")
	{
		// Declare parameters
		this->declare_parameter("robot_model", "m");
		this->declare_parameter("port", "/dev/ttyS0");
		this->declare_parameter("m_bEKF_option", false);
		this->declare_parameter("wheel_radius", 0.0);
		this->declare_parameter("wheel_distance", 0.0);
		this->declare_parameter("tread_width", 0.0);
		this->declare_parameter("cmd_vel_timeout", 0.5);
		// 속도 상한. 기본값은 Nav2 velocity smoother 의 최대값(2.0 m/s, 2.0 rad/s)이라 평소 주행에는 영향이 없고,
		// 조이스틱 누적값이나 잘못된 명령이 보드 한계(약 32 m/s)까지 그대로 가는 것만 막는다
		this->declare_parameter("max_linear_velocity", 2.0);
		this->declare_parameter("max_angular_velocity", 2.0);

		// Get parameters
		std::string robot_model = this->get_parameter("robot_model").as_string();
		std::string port_str = this->get_parameter("port").as_string();
		strncpy(port, port_str.c_str(), sizeof(port) - 1);
		m_bEKF_option = this->get_parameter("m_bEKF_option").as_bool();
		m_dCmdVelTimeout = this->get_parameter("cmd_vel_timeout").as_double();
		m_dMaxLinearVelocity = std::fabs(this->get_parameter("max_linear_velocity").as_double());
		m_dMaxAngularVelocity = std::fabs(this->get_parameter("max_angular_velocity").as_double());

		// Configure physical parameters based on robot_model
		if(robot_model == "h" || robot_model == "H")
		{
			m_dWheel_Radius = 0.0850;
			m_dWheel_Distance = 0.5194;
			m_dTread_Width = 0.04;
		}
		else if(robot_model == "edu" || robot_model == "EDU")
		{
			m_dWheel_Radius = 0.0750;
			m_dWheel_Distance = 0.3950;
			m_dTread_Width = 0.04;
		}
		else if(robot_model == "mh" || robot_model == "MH")
		{
			m_dWheel_Radius = 0.0850;
			m_dWheel_Distance = 0.5194;
			m_dTread_Width = 0.04;
		}
		else if(robot_model == "s" || robot_model == "S")
		{
			m_dWheel_Radius = 0.1015;
			m_dWheel_Distance = 0.3770;
			m_dTread_Width = 0.04;
		}
		else if(robot_model == "sm" || robot_model == "SM")
		{
			m_dWheel_Radius = 0.0735;
			m_dWheel_Distance = 0.3884;
			m_dTread_Width = 0.04;
		}
		else // default "m"
		{
			m_dWheel_Radius = 0.1033;
			m_dWheel_Distance = 0.4330;
			m_dTread_Width = 0.04;
		}

		// User overrides
		double custom_wheel_radius = this->get_parameter("wheel_radius").as_double();
		double custom_wheel_distance = this->get_parameter("wheel_distance").as_double();
		double custom_tread_width = this->get_parameter("tread_width").as_double();
		if(custom_wheel_radius > 0.0) m_dWheel_Radius = custom_wheel_radius;
		if(custom_wheel_distance > 0.0) m_dWheel_Distance = custom_wheel_distance;
		if(custom_tread_width > 0.0) m_dTread_Width = custom_tread_width;

		RCLCPP_INFO(this->get_logger(), "TETRA Wheel Driver Initialized (Model: %s, Wheel Radius: %.4fm, Wheel Dist: %.4fm, EKF: %d)",
			robot_model.c_str(), m_dWheel_Radius, m_dWheel_Distance, m_bEKF_option);

		// tf2_ros
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		// publish list
		odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 30);
		bumper_publisher = this->create_publisher<std_msgs::msg::Int32>("bumper_data", 10);
		emg_publisher = this->create_publisher<std_msgs::msg::Int32>("emg_state", 1);
		left_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("left_error_code", 1);
		right_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("right_error_code", 1);
		// 상태 변화 시에만 발행하므로 transient_local 로 래치해서 늦게 붙는 관제 서버도 현재 상태를 받게 한다.
		drive_board_incompatible_publisher = this->create_publisher<std_msgs::msg::Bool>(
			"drive_board_incompatible", rclcpp::QoS(1).transient_local().reliable());

		// subscribe list
		joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&TETRAWheelDriver::joyCallback, this, _1));
		cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", rclcpp::SensorDataQoS(), std::bind(&TETRAWheelDriver::velCallback, this, _1));
		cmd_vel_subscriber2 = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav2", rclcpp::SensorDataQoS(), std::bind(&TETRAWheelDriver::velCallback2, this, _1));
		cmd_vel_manual_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_manual", rclcpp::SensorDataQoS(), std::bind(&TETRAWheelDriver::Manual_velCallback, this, _1));
		acc_subscriber = this->create_subscription<std_msgs::msg::Int32>("accel_vel", 10, std::bind(&TETRAWheelDriver::accelCallback, this, _1));
		pose_reset_subscriber = this->create_subscription<std_msgs::msg::Int32>("pose_reset", 10, std::bind(&TETRAWheelDriver::pose_resetCallback, this, _1));
		servo_on_subscriber = this->create_subscription<std_msgs::msg::Int32>("servo_on", 10, std::bind(&TETRAWheelDriver::servo_onCallback, this, _1));
		power_status_subscriber = this->create_subscription<std_msgs::msg::Int32>("power_status", 10, std::bind(&TETRAWheelDriver::power_statusCallback, this, _1));

		// service list
		drive_version_read_srv = create_service<interfaces::srv::PowerVersionRead>(
			"drive_version_read_cmd",
			std::bind(&TETRAWheelDriver::Drive_Version_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		parameter_read_srv = create_service<interfaces::srv::ParameterRead>(
        	"param_read_cmd", 
			std::bind(&TETRAWheelDriver::Parameter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		parameter_write_srv = create_service<interfaces::srv::ParameterWrite>(
        	"param_write_cmd", 
			std::bind(&TETRAWheelDriver::Parameter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));

		set_move_mode_srv = create_service<interfaces::srv::SetMoveMode>(
        	"mode_change_cmd", 
			std::bind(&TETRAWheelDriver::Movemode_Change_Command, this, std::placeholders::_1, std::placeholders::_2));

		linear_position_move_srv = create_service<interfaces::srv::LinearPositionMove>(
        	"linear_move_cmd", 
			std::bind(&TETRAWheelDriver::Linear_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

		angular_position_move_srv = create_service<interfaces::srv::AngularPositionMove>(
        	"angular_move_cmd", 
			std::bind(&TETRAWheelDriver::Angular_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

		servo_srv = create_service<interfaces::srv::Servo>(
        	"servo_cmd", 
			std::bind(&TETRAWheelDriver::Servo_Command, this, std::placeholders::_1, std::placeholders::_2));
	}

	rclcpp::Time current_time, last_time;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	// Publishers
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bumper_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr emg_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_error_code_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_error_code_publisher;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_board_incompatible_publisher;

	// 웹 관제 연결이 끊기거나 내비게이션이 죽어도 마지막 속도 명령으로 계속 주행하지 않도록,
	// cmd_vel_timeout 초 동안 새 명령이 없으면 정지한다 (0 이하면 비활성).
	double m_dCmdVelTimeout = 0.5;
	std::chrono::steady_clock::time_point m_lastVelocityCommandTime = std::chrono::steady_clock::now();

	void MarkVelocityCommand()
	{
		m_lastVelocityCommandTime = std::chrono::steady_clock::now();
	}

	void StopIfVelocityCommandStale()
	{
		if(m_dCmdVelTimeout <= 0.0 || (linear == 0.0 && angular == 0.0)) return;
		std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - m_lastVelocityCommandTime;
		if(elapsed.count() < m_dCmdVelTimeout) return;

		std::lock_guard<std::mutex> lock(mutex_);
		linear = 0.0;
		angular = 0.0;
		m_bpending_ = false;
		RCLCPP_WARN(this->get_logger(), "속도 명령이 %.1f초 동안 없어 정지합니다", elapsed.count());
	}

	double m_dMaxLinearVelocity = 2.0;
	double m_dMaxAngularVelocity = 2.0;

	double ClampLinear(double value) const { return std::clamp(value, -m_dMaxLinearVelocity, m_dMaxLinearVelocity); }
	double ClampAngular(double value) const { return std::clamp(value, -m_dMaxAngularVelocity, m_dMaxAngularVelocity); }

	// 모터 드라이버 에러 처리.
	// 예전에는 에러가 보이면 매 주기(30Hz) 리셋하고 서보를 다시 켜서, 과부하·엔코더 고장이 계속돼도 모터에 힘을 계속 줬다.
	// 비상정지·저전압은 원인이 풀리면 다시 달려야 하므로 1초마다 리셋을 시도하고,
	// 그 밖의 고장은 3번까지만 시도한 뒤 서보를 끈 채 멈춰 두고 운영자가 서보를 다시 켤 때까지 기다린다.
	static constexpr double kFaultResetIntervalSec = 1.0;
	static constexpr double kFaultClearSec = 10.0;
	static constexpr int kMaxHardFaultResets = 3;
	int fault_reset_count_ = 0;
	bool fault_latched_ = false;
	int last_reported_left_error_ = kMotorErrorNone;
	int last_reported_right_error_ = kMotorErrorNone;
	// time_point::min() 으로 두면 now() - min() 이 넘쳐 음수가 되므로 충분히 지난 시각으로 시작한다
	std::chrono::steady_clock::time_point last_fault_reset_ = std::chrono::steady_clock::now() - std::chrono::hours(1);
	std::chrono::steady_clock::time_point last_fault_seen_ = std::chrono::steady_clock::now() - std::chrono::hours(1);

	static bool IsSelfClearingFault(int code)
	{
		return code == kMotorErrorNone || code == kMotorErrorEmergencyStop || code == kMotorErrorUnderVoltage;
	}

	static double SecondsSince(std::chrono::steady_clock::time_point since)
	{
		return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
	}

	void HandleMotorErrors(int left, int right)
	{
		if(left != last_reported_left_error_ || right != last_reported_right_error_)
		{
			PrintMotorErrorLog("Left", left);
			PrintMotorErrorLog("Right", right);
			last_reported_left_error_ = left;
			last_reported_right_error_ = right;
		}

		if(left == kMotorErrorNone && right == kMotorErrorNone)
		{
			// 리셋하면 보드가 에러를 잠시 지우므로, 한동안 깨끗해야 시도 횟수를 되돌린다
			if(fault_reset_count_ > 0 && SecondsSince(last_fault_seen_) > kFaultClearSec) fault_reset_count_ = 0;
			return;
		}

		last_fault_seen_ = std::chrono::steady_clock::now();
		if(fault_latched_ || SecondsSince(last_fault_reset_) < kFaultResetIntervalSec) return;

		const bool hard_fault = !IsSelfClearingFault(left) || !IsSelfClearingFault(right);
		if(hard_fault && fault_reset_count_ >= kMaxHardFaultResets)
		{
			fault_latched_ = true;
			dssp_rs232_drv_module_set_servo(0);
			RCLCPP_ERROR(this->get_logger(),
				"모터 드라이버 에러가 리셋 %d번 뒤에도 남아 서보를 끈 채 멈춥니다 (L 0x%X, R 0x%X). 원인을 해결한 뒤 서보를 다시 켜세요 (servo_on 1).",
				kMaxHardFaultResets, left, right);
			return;
		}

		dssp_rs232_drv_module_set_drive_err_reset();
		usleep(1000);
		dssp_rs232_drv_module_set_servo(1);
		last_fault_reset_ = std::chrono::steady_clock::now();
		if(hard_fault) fault_reset_count_++;
	}

	// 운영자가 서보를 켜면 멈춰 둔 고장 상태를 풀고 다시 시도한다
	void ClearFaultLatch()
	{
		if(fault_latched_) RCLCPP_INFO(this->get_logger(), "서보 ON 요청으로 모터 에러 정지 상태를 해제합니다");
		fault_latched_ = false;
		fault_reset_count_ = 0;
	}

	bool IsFaultLatched() const { return fault_latched_; }

	// 일시적인 노이즈로 오탐하지 않도록 구형 ACK 가 연속으로 들어올 때만 비호환으로 판정한다.
	static constexpr int kLegacyAckThreshold = 20;
	int legacy_ack_streak_ = 0;
	bool drive_board_incompatible_ = false;
	bool drive_board_state_published_ = false;

	void PublishDriveBoardIncompatible(bool incompatible)
	{
		if(drive_board_state_published_ && drive_board_incompatible_ == incompatible) return;
		drive_board_incompatible_ = incompatible;
		drive_board_state_published_ = true;

		std_msgs::msg::Bool msg;
		msg.data = incompatible;
		drive_board_incompatible_publisher->publish(msg);
		if(incompatible) {
			RCLCPP_ERROR(this->get_logger(),
				"구동보드가 velocity2 주행 명령을 처리하지 못했습니다 (FLAG_FAIL 응답 02 32 03). 구동보드 모드·에러 상태와 펌웨어 버전을 확인하세요.");
		} else {
			RCLCPP_INFO(this->get_logger(), "구동보드 velocity2 응답 정상");
		}
	}

	void UpdateDriveBoardCompatibility()
	{
		switch(drvm_get_velocity2_response()) {
			case DRVM_RESP_OK:
				legacy_ack_streak_ = 0;
				PublishDriveBoardIncompatible(false);
				break;
			case DRVM_RESP_LEGACY_ACK:
				if(++legacy_ack_streak_ >= kLegacyAckThreshold) PublishDriveBoardIncompatible(true);
				break;
			default:
				// 무응답(보드 분리 등)은 보드 버전과 무관하므로 판정을 바꾸지 않는다.
				break;
		}
	}

	// Subscriptions
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber2;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_manual_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr acc_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pose_reset_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_on_subscriber;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr power_status_subscriber;

	// Kinematics functions
	double distance_to_wheelrotation(double wheel_diameter, double distance)
	{
		return (distance / (M_PI * wheel_diameter));
	}

	double direction_to_diwheelrotation_diff(double wheel_distance, double wheel_diameter, double rad)
	{
		return (wheel_distance / (wheel_diameter * M_PI)) * rad;
	}

	void speed_to_diwheel_rpm(double meter_per_sec, double rad_per_sec, double * left_rpm, double * right_rpm)
	{
		double linearRotate = distance_to_wheelrotation(2.0 * m_dWheel_Radius, meter_per_sec);
		double angularRorate = direction_to_diwheelrotation_diff(m_dWheel_Distance / 2.0, 2.0 * m_dWheel_Radius, rad_per_sec);

		*left_rpm = (linearRotate - angularRorate) * 60.0;
		*right_rpm = (linearRotate + angularRorate) * 60.0;
	}

	double RPM_to_ms(double d_rpm)
	{
		return m_dWheel_Radius * 2.0 * M_PI * (d_rpm / 60.0);
	}

	void SetMoveCommand(double fLinear_vel, double fAngular_vel)
	{
		double Left_Wheel_vel = 0.0;
		double Right_Wheel_vel = 0.0;
		speed_to_diwheel_rpm(fLinear_vel, fAngular_vel, &Left_Wheel_vel, &Right_Wheel_vel);

		int iData1 = 1000.0 * RPM_to_ms(Left_Wheel_vel);
		int iData2 = 1000.0 * RPM_to_ms(Right_Wheel_vel);
		int ret = dssp_rs232_drv_module_set_velocity2(iData1, iData2, &m_dX_pos, &m_dY_pos, &m_dTheta, &m_dVelocity_l, &m_dVelocity_r, &m_bumper_data, &m_emg_state);
		UpdateDriveBoardCompatibility();
		if(ret == -2) dssp_rs232_drv_module_read_bumper_emg(&m_bumper_data, &m_emg_state, &m_left_error_code, &m_right_error_code);
		else return;
		
		// 에러가 있으면 일단 서보를 끈다. 다시 켤지는 메인 루프의 HandleMotorErrors 가 정한다
		if(m_left_error_code != kMotorErrorNone || m_right_error_code != kMotorErrorNone)
		{
			usleep(1000);
			dssp_rs232_drv_module_set_servo(0); // Servo Off
		}
	}
	
	// Callbacks
	void velCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if(m_bpending_) return;
		linear = vel->linear.x;
		angular = vel->angular.z;
		MarkVelocityCommand();
	}

	void velCallback2(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		m_bpending_ = true;
		linear = vel->linear.x;
		angular = vel->angular.z;
		MarkVelocityCommand();

		if(linear == 0.0 && angular == 0.0)
		{
			m_bpending_ = false;
		}
	}

	void Manual_velCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		linear = vel->linear.x;
		angular = vel->angular.z;
		MarkVelocityCommand();
	}
	
	void accelCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		int m_iAccel_data = msg->data;
		if(m_old_accel_data != m_iAccel_data)
		{
			dssp_rs232_drv_module_set_parameter(6, m_iAccel_data);
			m_old_accel_data = m_iAccel_data;
		}
	}

	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
	{
		if(joy->buttons.size() > 9)
		{
			if(joy->buttons[8])
			{
				m_bFlag_joy_enable = false;
			}
			if(joy->buttons[9])
			{
				m_bFlag_joy_enable = true;
			}
		}

		if(m_bFlag_joy_enable)
		{
			if(joy->axes.size() > 5)
			{
				if(joy->axes[4])
				{
					bt_angular += joy_angular * joy->axes[4];
				}
				if(joy->axes[5])
				{
					bt_linear += joy_linear * joy->axes[5];
				}
			}

			if(joy->buttons.size() > 5)
			{
				if(joy->buttons[0])
				{
					joy_angular -= 1.0;
					if(joy_angular < 0) joy_angular = 0;
				}
				if(joy->buttons[1])
				{
					joy_linear -= 1.0;
					if(joy_linear < 0) joy_linear = 0;
				}
				if(joy->buttons[2])
				{
					joy_angular += 1.0;
				}
				if(joy->buttons[3])
				{
					joy_linear += 1.0;
				}
				if(joy->buttons[5])
				{
					bt_linear = 0;
					bt_angular = 0;
					joy_linear = 1.0;
					joy_angular = 1.0;
				}
			}
			
			if(joy->axes.size() > 2)
			{
				linear = (joy_linear * (double)joy->axes[1] + bt_linear) / 3.0;
				angular = ((double)joy_angular * (joy->axes[1] >= 0 ? joy->axes[2] : (joy->axes[2] * -1)) + bt_angular) / 3.0;
				MarkVelocityCommand();
			}
		}
	}

	void pose_resetCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		Reset = msg->data;
		if(Reset == 1)
		{
			dssp_rs232_drv_module_reset_odometry();
			Reset = 0;
		}
	}

	void servo_onCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		int m_idata = msg->data;
		if(m_idata == 1)
		{
			ClearFaultLatch();
			bPosition_mode_flag = false;
			dssp_rs232_drv_module_set_velocitymode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			m_idata = 0;
		}
		else if(m_idata == 2)
		{
			dssp_rs232_drv_module_set_servo(0);
			m_idata = 0;
		}
		else if(m_idata == 3)
		{
			ClearFaultLatch();
			bPosition_mode_flag = true;
			dssp_rs232_drv_module_set_positionmode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			m_idata = 0;
		}
	}

	void power_statusCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		int m_idata = msg->data;
		if(m_idata == -10)
		{
			dssp_rs232_drv_module_set_servo(0);
			printf("[Error]: Power Board Error !!! \n");
		}
	}

private:
	rclcpp::Service<interfaces::srv::ParameterRead>::SharedPtr parameter_read_srv;
	rclcpp::Service<interfaces::srv::PowerVersionRead>::SharedPtr drive_version_read_srv;
	rclcpp::Service<interfaces::srv::ParameterWrite>::SharedPtr parameter_write_srv;
	rclcpp::Service<interfaces::srv::SetMoveMode>::SharedPtr set_move_mode_srv;
	rclcpp::Service<interfaces::srv::LinearPositionMove>::SharedPtr linear_position_move_srv;
	rclcpp::Service<interfaces::srv::AngularPositionMove>::SharedPtr angular_position_move_srv;
	rclcpp::Service<interfaces::srv::Servo>::SharedPtr servo_srv;

	// 응답 문자열 "12605130" = 보드 종류 1, 2026-05-13, 버전 0 → 필드마다 숫자 한 자리씩 담는다
	bool Drive_Version_Read_Command(const std::shared_ptr<interfaces::srv::PowerVersionRead::Request> request, const std::shared_ptr<interfaces::srv::PowerVersionRead::Response> response)
	{
		(void)request;
		char version[DRVM_VERSION_LENGTH + 1] = {0};
		response->command_result = (dssp_rs232_drv_module_read_version(version) == 0);
		if(!response->command_result) return true;

		response->d0_board_type = version[0] - '0';
		response->d1_year2 = version[1] - '0';
		response->d2_year1 = version[2] - '0';
		response->d3_month2 = version[3] - '0';
		response->d4_month1 = version[4] - '0';
		response->d5_day2 = version[5] - '0';
		response->d6_day1 = version[6] - '0';
		response->d7_ver = version[7] - '0';
		return true;
	}

	bool Parameter_Read_Command(const std::shared_ptr<interfaces::srv::ParameterRead::Request> request, const std::shared_ptr<interfaces::srv::ParameterRead::Response> response)
	{
		response->command_result = (dssp_rs232_drv_module_read_parameter(request->num, &m_iParam) == 0);
		// 읽기에 실패하면 이전 값이 남아 있으므로 돌려주지 않는다
		response->data = response->command_result ? m_iParam : 0;
		return true;
	}

	bool Parameter_Write_Command(const std::shared_ptr<interfaces::srv::ParameterWrite::Request> request, const std::shared_ptr<interfaces::srv::ParameterWrite::Response> response)
	{
		dssp_rs232_drv_module_set_servo(0);
		usleep(10000);
		const int ret = dssp_rs232_drv_module_set_parameter(request->num, request->data);
		usleep(10000);
		// 모터 에러로 멈춰 둔 상태면 파라미터를 바꿨다고 서보를 다시 켜지 않는다
		if(!IsFaultLatched()) dssp_rs232_drv_module_set_servo(1);
		usleep(10000);
		response->command_result = (ret == 0);
		return true;
	}

	bool Movemode_Change_Command(const std::shared_ptr<interfaces::srv::SetMoveMode::Request> request, const std::shared_ptr<interfaces::srv::SetMoveMode::Response> response)
	{
		int ret = 0;
		if(request->mode == 1) // Position Mode
		{			
			bPosition_mode_flag = true;
			dssp_rs232_drv_module_set_servo(0);
			usleep(10000);
			ret = dssp_rs232_drv_module_set_positionmode();
		}
		else // Velocity Mode
		{
			bPosition_mode_flag = false;
			dssp_rs232_drv_module_set_servo(0);
			usleep(10000);
			ret = dssp_rs232_drv_module_set_velocitymode();
		}
		usleep(10000);
		if(!IsFaultLatched()) dssp_rs232_drv_module_set_servo(1);
		usleep(10000);
		response->command_result = (ret == 0);
		return true;
	}

	bool Linear_Move_Command(const std::shared_ptr<interfaces::srv::LinearPositionMove::Request> request, const std::shared_ptr<interfaces::srv::LinearPositionMove::Response> response)
	{
		response->command_result = (dssp_rs232_drv_module_set_position(0, 0, request->linear_position) == 0);
		return true;
	}

	bool Angular_Move_Command(const std::shared_ptr<interfaces::srv::AngularPositionMove::Request> request, const std::shared_ptr<interfaces::srv::AngularPositionMove::Response> response)
	{
		response->command_result = (dssp_rs232_drv_module_set_position(2, 0, request->angular_degree) == 0);
		return true;
	}

	bool Servo_Command(const std::shared_ptr<interfaces::srv::Servo::Request> request, const std::shared_ptr<interfaces::srv::Servo::Response> response)
	{
		int ret = 0;
		if(request->data == 1)
		{
			ClearFaultLatch();
			bPosition_mode_flag = false;
			dssp_rs232_drv_module_set_velocitymode();
			usleep(10000);
			ret = dssp_rs232_drv_module_set_servo(1);
		}
		else
		{
			ret = dssp_rs232_drv_module_set_servo(0);
		}
		response->command_result = (ret == 0);
		return true;
	}
};

// 보드 초기화. 펌웨어를 새로 올린 보드는 속도 모드가 아닌 상태로 부팅해 velocity2 를 FLAG_FAIL(02 32 03)로 거부하므로 서보 ON 전에 모드를 명시한다
static void InitDriveBoard()
{
	dssp_rs232_drv_module_set_drive_err_reset();
	usleep(10000);
	dssp_rs232_drv_module_set_parameter(6, m_old_accel_data);
	usleep(10000);
	dssp_rs232_drv_module_set_velocitymode();
	usleep(10000);
	dssp_rs232_drv_module_set_servo(1);
	usleep(10000);
}

static double NormalizeAngle(double angle)
{
	return std::atan2(std::sin(angle), std::cos(angle));
}

// 노드가 어떤 경로로 끝나든(정상 종료, 예외) 바퀴를 세운 뒤 서보를 끈다.
// 서보만 끄면 모터가 풀려 경사에서 굴러갈 수 있으므로 먼저 0 속도를 보낸다
struct StopMotorsOnExit
{
	~StopMotorsOnExit()
	{
		double x = 0.0, y = 0.0, theta = 0.0, vl = 0.0, vr = 0.0;
		int bumper = 0, emg = 0;
		dssp_rs232_drv_module_set_velocity2(0, 0, &x, &y, &theta, &vl, &vr, &bumper, &emg);
		usleep(10000);
		dssp_rs232_drv_module_set_servo(0);
		printf("Servo Off ! \n");
		usleep(10000);
		dssp_rs232_drv_module_destroy();
		printf("RS232 Disconnect ! \n");
	}
};

// 연속으로 이만큼 읽기에 실패하면(30Hz 기준 약 0.5초) 보드가 끊긴 것으로 보고 포트를 다시 연다
constexpr int kDisconnectedReadFailures = 15;
constexpr double kReconnectIntervalSec = 2.0;

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<TETRAWheelDriver>();

	if(port[0] == '\0')
	{
		sprintf(port, "/dev/ttyS0");
	}

	if(dssp_rs232_drv_module_create(port, 200) == 0)
	{
		printf("TETRA_Driver_rs232 Port (%s) Open Success\n", port);
	}
	else
	{
		printf("TETRA_Driver_rs232 Port (%s) Open Error!\n", port);
		return -1;
	}

	StopMotorsOnExit stop_motors_on_exit;
	InitDriveBoard();
	dssp_rs232_drv_module_reset_odometry();
	usleep(10000);
	rclcpp::WallRate loop_rate(30);
	int consecutive_read_failures = 0;
	bool board_disconnected = false;
	auto last_reconnect_attempt = std::chrono::steady_clock::now();

	printf("======================================\n");
	printf(" TETRA Wheel Driver Node Started (%s)\n", port);
	printf("======================================\n");

	// 루프 안에서 예외가 나도 스택이 풀려 StopMotorsOnExit 가 바퀴를 세우도록 main 밖으로 내보내지 않는다
	try
	{
		while (rclcpp::ok() && !stop_requested)
		{
			rclcpp::spin_some(node);
			node->StopIfVelocityCommandStale();

			// 조이스틱 버튼 누적값이 끝없이 커지지 않게 한다 (출력은 /3 해서 쓴다)
			bt_linear = std::clamp(bt_linear, -3.0 * node->m_dMaxLinearVelocity, 3.0 * node->m_dMaxLinearVelocity);
			bt_angular = std::clamp(bt_angular, -3.0 * node->m_dMaxAngularVelocity, 3.0 * node->m_dMaxAngularVelocity);
			input_linear = node->ClampLinear(linear);
			input_angular = node->ClampAngular(angular);

			if(input_linear == 0.0)
			{
				if(input_linear < control_linear)
				{
					control_linear = max(input_linear, control_linear - 0.05);
				}
				else if(input_linear > control_linear)
				{
					control_linear = min(input_linear, control_linear + 0.05);
				}
				else
				{
					control_linear = input_linear;
				}
			}
			else
			{
				if(input_linear < control_linear)
				{
					control_linear = max(input_linear, control_linear - 0.01);
					if(control_linear > 0)
					{
						control_linear = max(input_linear, control_linear - 0.05);
					}
				}
				else if(input_linear > control_linear)
				{
					control_linear = min(input_linear, control_linear + 0.05);
				}
				else
				{
					control_linear = input_linear;
				}
			}
			control_angular = input_angular;

			// EMG Check Loop
			std_msgs::msg::Int32 emg_state;
			emg_state.data = m_emg_state;
			node->emg_publisher->publish(emg_state);

			// Bumper Check Loop
			std_msgs::msg::Int32 bumper_data;
			bumper_data.data = m_bumper_data;
			node->bumper_publisher->publish(bumper_data);
		
			// Error Code Check Loop
			std_msgs::msg::Int32 left_error_code;
			std_msgs::msg::Int32 right_error_code;
			left_error_code.data = m_left_error_code;
			right_error_code.data = m_right_error_code;
			node->left_error_code_publisher->publish(left_error_code);
			node->right_error_code_publisher->publish(right_error_code);

			// Odometry calculation
			const bool odometry_ok = (dssp_rs232_drv_module_read_odometry(&m_dX_pos, &m_dY_pos, &m_dTheta) == 0);
			if(odometry_ok)
			{
				if(board_disconnected) RCLCPP_INFO(node->get_logger(), "구동보드 통신이 복구되었습니다");
				consecutive_read_failures = 0;
				board_disconnected = false;
			}
			else if(++consecutive_read_failures >= kDisconnectedReadFailures)
			{
				if(!board_disconnected)
				{
					RCLCPP_ERROR(node->get_logger(), "구동보드 응답이 없습니다 (%s). 오도메트리 발행을 멈추고 포트를 다시 엽니다", port);
					board_disconnected = true;
				}
				// USB 를 뺐다 꽂으면 예전 fd 로는 계속 실패하므로 포트를 새로 연다
				if(std::chrono::duration<double>(std::chrono::steady_clock::now() - last_reconnect_attempt).count() >= kReconnectIntervalSec)
				{
					last_reconnect_attempt = std::chrono::steady_clock::now();
					if(dssp_rs232_drv_module_create(port, 200) == 0)
					{
						InitDriveBoard();
						// 보드가 다시 켜졌다면 오도메트리가 0 부터 다시 시작하므로 속도 계산 기준을 새로 잡는다
						first = true;
					}
				}
			}
			coordinates[0] = (m_dX_pos / 1000.0);
			coordinates[1] = (m_dY_pos / 1000.0);
			coordinates[2] = m_dTheta * (M_PI / 1800.0);

			if(!bPosition_mode_flag)
			{
				node->SetMoveCommand(control_linear, control_angular);
				dssp_rs232_drv_module_read_bumper_emg(&m_bumper_data, &m_emg_state, &m_left_error_code, &m_right_error_code);
			}

			node->HandleMotorErrors(m_left_error_code, m_right_error_code);

			if(IsServoEnabled(m_bumper_data)) 
			{
				if(m_bflag_bumper)
				{
					printf("!! SERVO ON !! \n");
					dssp_rs232_drv_module_set_servo(1);
					m_bflag_bumper = false;
				}
			}
			else
			{
				if(!m_bflag_bumper)
				{
					printf("!! SERVO OFF !! \n");
					dssp_rs232_drv_module_set_servo(0);
					usleep(1000);
					dssp_rs232_drv_module_set_drive_err_reset();
					usleep(1000);
					m_bflag_bumper = true;
				}
			}

			if(!odometry_ok)
			{
				// 읽지 못한 주기에 예전 위치를 새 시각으로 내보내면 EKF 가 멈춰 있는 것으로 착각한다
			}
			else if(first) 
			{
				node->current_time = node->now();
				node->last_time = node->current_time;
				for(int i = 0; i < 3; i++) prev_coordinates[i] = coordinates[i];
				first = false;
			}
			else 
			{
				node->current_time = node->now();
				dt = (node->current_time - node->last_time).seconds();
				if(dt > 0.0001)
				{
					// twist 는 child_frame_id(base_footprint) 기준이어야 한다. odom 좌표 변화량을 로봇 기준으로 돌린다.
					// 예전에는 odom 기준 dx/dt, dy/dt 를 그대로 넣어 로봇이 90° 방향일 때 EKF 가 옆으로 가는 것으로 받았다
					const double dx = coordinates[0] - prev_coordinates[0];
					const double dy = coordinates[1] - prev_coordinates[1];
					// 각도가 359.9° → 0° 로 넘어갈 때 한 주기 각속도가 튀지 않도록 차이를 -π~π 로 맞춘다
					const double dtheta = NormalizeAngle(coordinates[2] - prev_coordinates[2]);
					const double heading = coordinates[2];
					velocity[0] = (std::cos(heading) * dx + std::sin(heading) * dy) / dt;
					velocity[1] = (-std::sin(heading) * dx + std::cos(heading) * dy) / dt;
					velocity[2] = dtheta / dt;
					for(int i = 0; i < 3; i++) prev_coordinates[i] = coordinates[i];
				}

				tf2::Quaternion q;
				q.setRPY(0.0, 0.0, coordinates[2]);

				if(!m_bEKF_option)
				{
					geometry_msgs::msg::TransformStamped odom_trans;
					odom_trans.header.stamp = node->current_time;
					odom_trans.header.frame_id = "odom";
					odom_trans.child_frame_id = "base_footprint";
					odom_trans.transform.translation.x = coordinates[0];
					odom_trans.transform.translation.y = coordinates[1];
					odom_trans.transform.translation.z = 0.0;
					odom_trans.transform.rotation.x = q.x();
					odom_trans.transform.rotation.y = q.y();
					odom_trans.transform.rotation.z = q.z();
					odom_trans.transform.rotation.w = q.w();
					node->tf_broadcaster_->sendTransform(odom_trans);
				}
			
				nav_msgs::msg::Odometry odom;
				odom.header.stamp = node->current_time;
				odom.header.frame_id = "odom";
				odom.child_frame_id = "base_footprint";
				odom.pose.pose.position.x = coordinates[0];
				odom.pose.pose.position.y = coordinates[1];
				odom.pose.pose.position.z = 0.0;
				odom.pose.pose.orientation.x = q.x();
				odom.pose.pose.orientation.y = q.y();
				odom.pose.pose.orientation.z = q.z();
				odom.pose.pose.orientation.w = q.w();
				odom.twist.twist.linear.x = velocity[0];
				odom.twist.twist.linear.y = velocity[1];
				odom.twist.twist.linear.z = 0.0;
				odom.twist.twist.angular.z = velocity[2];
				node->odom_publisher->publish(odom);

				node->last_time = node->current_time;
			}

			loop_rate.sleep();
	    }
	}
	catch(const std::exception & e)
	{
		RCLCPP_FATAL(node->get_logger(), "예외로 주행 루프를 멈춥니다: %s", e.what());
	}

	printf("Shutting down TETRA Wheel Driver...\n");
	rclcpp::shutdown();
    return 0;
}
