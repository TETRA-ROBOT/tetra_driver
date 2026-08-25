#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
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

void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}

class TETRA: public rclcpp::Node
{
public:
	TETRA() : Node("tetra")
	{
		// Declare parameters
		this->declare_parameter("robot_model", "m");
		this->declare_parameter("port", "/dev/ttyS0");
		this->declare_parameter("m_bEKF_option", false);
		this->declare_parameter("wheel_radius", 0.0);
		this->declare_parameter("wheel_distance", 0.0);
		this->declare_parameter("tread_width", 0.0);

		// Get parameters
		std::string robot_model = this->get_parameter("robot_model").as_string();
		std::string port_str = this->get_parameter("port").as_string();
		strncpy(port, port_str.c_str(), sizeof(port) - 1);
		m_bEKF_option = this->get_parameter("m_bEKF_option").as_bool();

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

		RCLCPP_INFO(this->get_logger(), "TETRA Driver Initialized (Model: %s, Wheel Radius: %.4fm, Wheel Dist: %.4fm, EKF: %d)",
			robot_model.c_str(), m_dWheel_Radius, m_dWheel_Distance, m_bEKF_option);

		// tf2_ros
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		// publish list
		odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 30);
		bumper_publisher = this->create_publisher<std_msgs::msg::Int32>("bumper_data", 10);
		emg_publisher = this->create_publisher<std_msgs::msg::Int32>("emg_state", 1);
		left_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("left_error_code", 1);
		right_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("right_error_code", 1);

		// subscribe list
		joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&TETRA::joyCallback, this, _1));
		cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", rclcpp::SensorDataQoS(), std::bind(&TETRA::velCallback, this, _1));
		cmd_vel_subscriber2 = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav2", rclcpp::SensorDataQoS(), std::bind(&TETRA::velCallback2, this, _1));
		cmd_vel_manual_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_manual", rclcpp::SensorDataQoS(), std::bind(&TETRA::Manual_velCallback, this, _1));
		acc_subscriber = this->create_subscription<std_msgs::msg::Int32>("accel_vel", 10, std::bind(&TETRA::accelCallback, this, _1));
		pose_reset_subscriber = this->create_subscription<std_msgs::msg::Int32>("pose_reset", 10, std::bind(&TETRA::pose_resetCallback, this, _1));
		servo_on_subscriber = this->create_subscription<std_msgs::msg::Int32>("servo_on", 10, std::bind(&TETRA::servo_onCallback, this, _1));
		power_status_subscriber = this->create_subscription<std_msgs::msg::Int32>("power_status", 10, std::bind(&TETRA::power_statusCallback, this, _1));

		// service list
		parameter_read_srv = create_service<interfaces::srv::ParameterRead>(
        	"param_read_cmd", 
			std::bind(&TETRA::Parameter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));

		parameter_write_srv = create_service<interfaces::srv::ParameterWrite>(
        	"param_write_cmd", 
			std::bind(&TETRA::Parameter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));

		set_move_mode_srv = create_service<interfaces::srv::SetMoveMode>(
        	"mode_change_cmd", 
			std::bind(&TETRA::Movemode_Change_Command, this, std::placeholders::_1, std::placeholders::_2));

		linear_position_move_srv = create_service<interfaces::srv::LinearPositionMove>(
        	"linear_move_cmd", 
			std::bind(&TETRA::Linear_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

		angular_position_move_srv = create_service<interfaces::srv::AngularPositionMove>(
        	"angular_move_cmd", 
			std::bind(&TETRA::Angular_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

		servo_srv = create_service<interfaces::srv::Servo>(
        	"servo_cmd", 
			std::bind(&TETRA::Servo_Command, this, std::placeholders::_1, std::placeholders::_2));
	}

	rclcpp::Time current_time, last_time;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	// Publishers
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bumper_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr emg_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_error_code_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_error_code_publisher;

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
		if(ret == -2) dssp_rs232_drv_module_read_bumper_emg(&m_bumper_data, &m_emg_state, &m_left_error_code, &m_right_error_code);
		else return;
		
		// Error Code Check -> Reset & servo On Loop
		if(m_left_error_code != 48 || m_right_error_code != 48)
		{
			if(m_left_error_code != 48) {
				PrintMotorErrorLog("Left", m_left_error_code);
			}
			if(m_right_error_code != 48) {
				PrintMotorErrorLog("Right", m_right_error_code);
			}

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
	}

	void velCallback2(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		m_bpending_ = true;
		linear = vel->linear.x;
		angular = vel->angular.z;

		if(linear == 0.0 && angular == 0.0)
		{
			m_bpending_ = false;
		}
	}

	void Manual_velCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
	{
		linear = vel->linear.x;
		angular = vel->angular.z;
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
	rclcpp::Service<interfaces::srv::ParameterWrite>::SharedPtr parameter_write_srv;
	rclcpp::Service<interfaces::srv::SetMoveMode>::SharedPtr set_move_mode_srv;
	rclcpp::Service<interfaces::srv::LinearPositionMove>::SharedPtr linear_position_move_srv;
	rclcpp::Service<interfaces::srv::AngularPositionMove>::SharedPtr angular_position_move_srv;
	rclcpp::Service<interfaces::srv::Servo>::SharedPtr servo_srv;

	bool Parameter_Read_Command(const std::shared_ptr<interfaces::srv::ParameterRead::Request> request, const std::shared_ptr<interfaces::srv::ParameterRead::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_read_parameter(request->num, &m_iParam);
		response->data = m_iParam;
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Parameter_Write_Command(const std::shared_ptr<interfaces::srv::ParameterWrite::Request> request, const std::shared_ptr<interfaces::srv::ParameterWrite::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_set_servo(0);
		usleep(10000);
		dssp_rs232_drv_module_set_parameter(request->num, request->data);
		usleep(10000);
		dssp_rs232_drv_module_set_servo(1);
		usleep(10000);
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Movemode_Change_Command(const std::shared_ptr<interfaces::srv::SetMoveMode::Request> request, const std::shared_ptr<interfaces::srv::SetMoveMode::Response> response)
	{
		bool bResult = false;
		if(request->mode == 1) // Position Mode
		{			
			bPosition_mode_flag = true;
			dssp_rs232_drv_module_set_servo(0);
			usleep(10000);
			dssp_rs232_drv_module_set_positionmode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			usleep(10000);
		}
		else // Velocity Mode
		{
			bPosition_mode_flag = false;
			dssp_rs232_drv_module_set_servo(0);
			usleep(10000);
			dssp_rs232_drv_module_set_velocitymode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
			usleep(10000);
		}
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Linear_Move_Command(const std::shared_ptr<interfaces::srv::LinearPositionMove::Request> request, const std::shared_ptr<interfaces::srv::LinearPositionMove::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_set_position(0, 0, request->linear_position);
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Angular_Move_Command(const std::shared_ptr<interfaces::srv::AngularPositionMove::Request> request, const std::shared_ptr<interfaces::srv::AngularPositionMove::Response> response)
	{
		bool bResult = false;
		dssp_rs232_drv_module_set_position(2, 0, request->angular_degree);
		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Servo_Command(const std::shared_ptr<interfaces::srv::Servo::Request> request, const std::shared_ptr<interfaces::srv::Servo::Response> response)
	{
		bool bResult = false;
		int m_idata = request->data;
		if(m_idata == 1)
		{
			bPosition_mode_flag = false;
			dssp_rs232_drv_module_set_velocitymode();
			usleep(10000);
			dssp_rs232_drv_module_set_servo(1);
		}
		else
		{
			dssp_rs232_drv_module_set_servo(0);
		}
		bResult = true;
		response->command_result = bResult;
		return true;
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<TETRA>();

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

	// Servo On
	dssp_rs232_drv_module_set_servo(1);
	dssp_rs232_drv_module_set_parameter(6, m_old_accel_data);
	rclcpp::WallRate loop_rate(30);

	printf("======================================\n");
	printf(" TETRA Motor Driver Node Started (%s)\n", port);
	printf("======================================\n");

	while (rclcpp::ok() && !stop_requested)
	{
		rclcpp::spin_some(node);

		input_linear = linear;
		input_angular = angular;

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
		dssp_rs232_drv_module_read_odometry(&m_dX_pos, &m_dY_pos, &m_dTheta);
		coordinates[0] = (m_dX_pos / 1000.0);
		coordinates[1] = (m_dY_pos / 1000.0);
		coordinates[2] = m_dTheta * (M_PI / 1800.0);

		if(!bPosition_mode_flag)
		{
			node->SetMoveCommand(control_linear, control_angular);
			dssp_rs232_drv_module_read_bumper_emg(&m_bumper_data, &m_emg_state, &m_left_error_code, &m_right_error_code);
		}

		if(m_left_error_code != 48 || m_right_error_code != 48)
		{
			if(m_left_error_code != 48) {
				PrintMotorErrorLog("Left", m_left_error_code);
			}
			if(m_right_error_code != 48) {
				PrintMotorErrorLog("Right", m_right_error_code);
			}
			dssp_rs232_drv_module_set_drive_err_reset();
			usleep(1000);
			dssp_rs232_drv_module_set_servo(1);
		}

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

		if(first) 
		{
			node->current_time = node->now();
			node->last_time = node->current_time;
			first = false;
		}
		else 
		{
			node->current_time = node->now();
			dt = (node->current_time - node->last_time).seconds();
			if(dt > 0.0001)
			{
				for(int i = 0; i < 3; i++)
				{
					velocity[i] = (coordinates[i] - prev_coordinates[i]) / dt;
					prev_coordinates[i] = coordinates[i];
				}
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

	// Servo Off
	dssp_rs232_drv_module_set_servo(0);
	printf("Servo Off ! \n");
	usleep(10000);
	// RS232 Disconnect
	dssp_rs232_drv_module_destroy();
	printf("RS232 Disconnect ! \n");

	rclcpp::shutdown();
    return 0;
}
