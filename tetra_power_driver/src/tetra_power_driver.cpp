#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <array>
#include <vector>
#include <atomic>
#include <memory>
#include <string>

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// Custom Message
#include "interfaces/msg/gpio.hpp"
// Custom Service
#include "interfaces/srv/led_control.hpp"
#include "interfaces/srv/led_toggle_control.hpp"
#include "interfaces/srv/toggle_on.hpp"
#include "interfaces/srv/integrallog.hpp"
#include "interfaces/srv/power_set_outport.hpp" 
#include "interfaces/srv/power_get_io_status.hpp"
#include "interfaces/srv/power_set_enable.hpp"
#include "interfaces/srv/power_set_single_outport.hpp"
#include "interfaces/srv/power_set_single_enable.hpp"
#include "interfaces/srv/power_wheel_enable.hpp"
#include "interfaces/srv/power_parameter_read.hpp"
#include "interfaces/srv/power_parameter_write.hpp"
#include "interfaces/srv/power_data_read.hpp"
#include "interfaces/srv/power_adc_read.hpp"
#include "interfaces/srv/power_version_read.hpp"
#include "interfaces/srv/conveyor_auto_movement.hpp"
#include "interfaces/srv/conveyor_manual_movement.hpp"
#include "interfaces/srv/conveyor_parameter_read.hpp"
#include "interfaces/srv/conveyor_parameter_write.hpp"
#include "interfaces/srv/conveyor_data_read.hpp"
#include "interfaces/srv/power_sonar_read.hpp"
#include "interfaces/srv/power_sonar_cmd.hpp"
#include "interfaces/srv/board_raw_command.hpp"
#include "tetra_serial/raw_command_service.hpp"
#include "tetra_serial/renamed_parameter.hpp"

// RS232 Power Module Driver
#ifdef __cplusplus
extern "C" {
#endif
#include "power_module.h"
#include "dssp_rs232_power_module.h"
#ifdef __cplusplus
}
#endif

#define BUF_LEN 4096
#define Ultrasonic_MIN_range 0.04

using namespace std;
using namespace std::chrono_literals;

// Global Parameters
int m_iParam0 = 0;
int m_iParam1 = 0;
int m_iParam2 = 0;
int m_iParam3 = 0;
int m_iParam4 = 0;
int m_iParam5 = 0;
int m_iParam6 = 0;
int m_iParam7 = 0;
int m_iParam8 = 0;
int m_iParam9 = 0;
int m_iParam10 = 0;
int m_iParam11 = 0;
int m_iParam12 = 0;
int m_iParam13 = 0;
int m_iParam14 = 0;
int m_iParam15 = 0;
int m_iParam16 = 0;
int m_iParam17 = 0;
int m_iParam18 = 0;
int m_iParam19 = 0;
int m_iParam20 = 0;
int m_iParam21 = 0;
int m_iParam22 = 0;
int m_iParam23 = 0;
int m_iParam24 = 0;
int m_iParam25 = 0;
int m_iParam26 = 0;
int m_iParam27 = 0;
int m_iParam28 = 0;
int m_iParam29 = 0;
int m_iParam30 = 0;
int m_iParam31 = 0;

// serial
int m_iFlag_PowerCheck_cnt = 0;
int com_port = 0;
char port[64] = {0,};

// battery data
int m_ibattery_Level = 0;
double m_dbattery = 0.0;
double m_dVoltage = 0.0;
double m_dCurrent = 0.0;
int m_imode_status = 0;
int m_iPowerCheck = 0;
int m_iPowerCheckCount = 0;

// Ultrasonic data
bool m_bUltrasonic_option = false;
// 전원 보드는 감지가 없으면 0 이 아니라 자기 최대 거리(보드 파라미터 p5_sonar_max_distance, 기본 500mm)를 보낸다.
// 이 값 이상을 장애물로 찍으면 센서마다 그 거리에 가짜 장애물이 항상 생긴다
double m_dUltrasonicMaxRange = 0.5;

// 감지가 없는 측정값(0 또는 최대 거리 이상)은 최대 거리로 맞춘다. RangeToCloud 는 최대 거리 미만만 점으로 만든다
static double UltrasonicRangeOrMax(double measured_range)
{
	const bool nothing_detected = (measured_range <= 0.0 || measured_range >= m_dUltrasonicMaxRange);
	return nothing_detected ? m_dUltrasonicMaxRange : measured_range;
}
double m_dUltrasonic[8] = {0.0, };

// 초음파 센서 4개. 순서는 보드가 보내는 순서(m_dUltrasonic[0..3])와 같다
struct SonarChannel
{
	const char* frame_id;
	const char* range_topic;
	const char* points_topic;
};
constexpr size_t kSonarCount = 4;
constexpr SonarChannel kSonarChannels[kSonarCount] = {
	{"sonar_DL", "ultrasonic_dl", "range_points_dl"},
	{"sonar_RL", "ultrasonic_rl", "range_points_rl"},
	{"sonar_RR", "ultrasonic_rr", "range_points_rr"},
	{"sonar_DR", "ultrasonic_dr", "range_points_dr"},
};
std::array<sensor_msgs::msg::Range, kSonarCount> range_msgs;

// GPIO data
interfaces::msg::Gpio gpio_msg;
// gpio_status 메시지는 포트마다 필드가 따로다 (input0 ~ input7, output0 ~ output7). 반복문으로 채우려고 필드 위치를 표로 둔다
constexpr size_t kGpioPortCount = 8;
using GpioField = uint8_t interfaces::msg::Gpio::*;
constexpr GpioField kGpioInputFields[kGpioPortCount] = {
	&interfaces::msg::Gpio::input0, &interfaces::msg::Gpio::input1, &interfaces::msg::Gpio::input2, &interfaces::msg::Gpio::input3,
	&interfaces::msg::Gpio::input4, &interfaces::msg::Gpio::input5, &interfaces::msg::Gpio::input6, &interfaces::msg::Gpio::input7,
};
constexpr GpioField kGpioOutputFields[kGpioPortCount] = {
	&interfaces::msg::Gpio::output0, &interfaces::msg::Gpio::output1, &interfaces::msg::Gpio::output2, &interfaces::msg::Gpio::output3,
	&interfaces::msg::Gpio::output4, &interfaces::msg::Gpio::output5, &interfaces::msg::Gpio::output6, &interfaces::msg::Gpio::output7,
};
int m_iOutput[8] = {0,};
int m_iInput[8] = {0,};

// Conveyor data
bool m_bConveyor_option = false;
// 리프트 모터 위치(보드 ML 명령, 0 = 가장 높음 ~ 764 = 가장 낮음)를 /lift_status 로 낸다. 리프트가 있는 MH 브링업이 켠다.
// 예전 MH 드라이버(legacy_archive/tetra_mh_interface.cpp)가 매 주기 냈고, 웹 관제 리프트 창이 이 값으로 높이를 보인다
bool m_bLift_position_option = false;
int m_iLift_position = 0;
int m_iLift_poll_count = 0;
// 30 Hz 주기 중 3 번에 한 번(10 Hz)만 읽는다: 읽을 때마다 보드 왕복이 하나 늘어 다른 폴링이 늦어진다
const int kLiftPollEvery = 3;
int m_dConveyor_sensor = 0; 
int m_iConveyor_movement = 0;

// File log
FILE *fp;
int status;
char Textbuffer[BUF_LEN];
char strPath[BUF_LEN];

std::atomic<bool> stop_requested(false);
// 시그널 핸들러 안에서는 printf 같은 함수를 쓰면 안 되므로 플래그만 세운다
void signal_handler(int signal) 
{
    (void)signal;
    stop_requested = true;
}

void Error_Log_write(string log_data)
{
	time_t timer; 
	struct tm* t; 
	timer = time(NULL); 
	t = localtime(&timer); 

	string m_colon = ":";
	string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
	string m_temp1 = m_time_stemp + "_error";		

	string m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
	fp = fopen(m_strFilePathName.c_str(), "w");
	if(fp != NULL) 
	{
		fprintf(fp, "[Error]: %s \n", log_data.c_str());
		fclose(fp);
	}
}

class TETRAPowerDriver: public rclcpp::Node
{
public:
	TETRAPowerDriver() : Node("tetra_power_driver")
	{
		// Declare and get parameters
		this->declare_parameter("port", "/dev/ttyS1");
		this->declare_parameter("ultrasonic_max_range", 0.5);

		std::string port_param = this->get_parameter("port").as_string();
		strncpy(port, port_param.c_str(), sizeof(port) - 1);
		m_bConveyor_option = tetra_serial::DeclareRenamedBool(*this, "conveyor_option", "m_bConveyor_option", false);
		m_bUltrasonic_option = tetra_serial::DeclareRenamedBool(*this, "ultrasonic_option", "m_bUltrasonic_option", false);
		m_bLift_position_option = this->declare_parameter("lift_position_option", false);
		m_dUltrasonicMaxRange = this->get_parameter("ultrasonic_max_range").as_double();

		// Publishers
		power_status_publisher = this->create_publisher<std_msgs::msg::Int32>("power_status", 1);
		servo_publisher = this->create_publisher<std_msgs::msg::Int32>("servo_on", 1);
		tetra_battery_publisher = this->create_publisher<std_msgs::msg::Int32>("tetra_battery", 1);
		battery_voltage_publisher = this->create_publisher<std_msgs::msg::Float64>("battery_voltage", 1);
		battery_current_publisher = this->create_publisher<std_msgs::msg::Float64>("battery_current", 1);
		docking_status_publisher = this->create_publisher<std_msgs::msg::Int32>("docking_status", 1);
		gpio_status_publisher = this->create_publisher<interfaces::msg::Gpio>("gpio_status", 10);

		// Ultrasonic Option Publishers
		for(size_t sonar = 0; sonar < kSonarCount; sonar++)
		{
			ultrasonic_publishers[sonar] = this->create_publisher<sensor_msgs::msg::Range>(kSonarChannels[sonar].range_topic, 10);
		}

		// Sonar range to PointCloud2
		for(size_t sonar = 0; sonar < kSonarCount; sonar++)
		{
			points_publishers[sonar] = this->create_publisher<sensor_msgs::msg::PointCloud2>(kSonarChannels[sonar].points_topic, 10);
		}

		// Conveyor Option Publishers
		conveyor_sensor_publisher = this->create_publisher<std_msgs::msg::Int32>("conveyor_sensor", 1);
		conveyor_movement_publisher = this->create_publisher<std_msgs::msg::Int32>("conveyor_movement", 1);
		lift_status_publisher = this->create_publisher<std_msgs::msg::Int32>("lift_status", 1);

		// Services. Power_*·Conveyor_* 는 예전 대문자 이름도 같이 연다 (legacy_services_). 새 코드는 소문자 이름을 쓴다
		led_control_srv = create_service<interfaces::srv::LedControl>(
        	"led_cmd", 
			std::bind(&TETRAPowerDriver::LED_Control_Command, this, std::placeholders::_1, std::placeholders::_2));

		led_toggle_control_srv = create_service<interfaces::srv::LedToggleControl>(
        	"led_toggle_cmd", 
			std::bind(&TETRAPowerDriver::LED_Toggle_Command, this, std::placeholders::_1, std::placeholders::_2));

		toggle_on_srv = create_service<interfaces::srv::ToggleOn>(
        	"turn_on_cmd", 
			std::bind(&TETRAPowerDriver::Toggle_On_Command, this, std::placeholders::_1, std::placeholders::_2));

		integrallog_srv = create_service<interfaces::srv::Integrallog>(
        	"log_cmd", 
			std::bind(&TETRAPowerDriver::Integrallog_Command, this, std::placeholders::_1, std::placeholders::_2));

		power_set_outport_srv = create_service<interfaces::srv::PowerSetOutport>(
        	"power_outport_cmd", 
			std::bind(&TETRAPowerDriver::Power_Set_Outport_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerSetOutport>("Power_outport_cmd", std::bind(&TETRAPowerDriver::Power_Set_Outport_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_set_single_outport_srv = create_service<interfaces::srv::PowerSetSingleOutport>(
        	"power_single_outport_cmd", 
			std::bind(&TETRAPowerDriver::Power_Set_Single_Outport_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerSetSingleOutport>("Power_single_outport_cmd", std::bind(&TETRAPowerDriver::Power_Set_Single_Outport_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_get_io_status_srv = create_service<interfaces::srv::PowerGetIoStatus>(
        	"power_io_status_cmd", 
			std::bind(&TETRAPowerDriver::Power_Get_IO_Status_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerGetIoStatus>("Power_io_status_cmd", std::bind(&TETRAPowerDriver::Power_Get_IO_Status_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_set_enable_srv = create_service<interfaces::srv::PowerSetEnable>(
        	"power_enable_cmd", 
			std::bind(&TETRAPowerDriver::Power_Set_Enable_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerSetEnable>("Power_enable_cmd", std::bind(&TETRAPowerDriver::Power_Set_Enable_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_set_single_enable_srv = create_service<interfaces::srv::PowerSetSingleEnable>(
        	"power_single_enable_cmd", 
			std::bind(&TETRAPowerDriver::Power_Set_Single_Enable_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerSetSingleEnable>("Power_single_enable_cmd", std::bind(&TETRAPowerDriver::Power_Set_Single_Enable_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_wheel_enable_srv = create_service<interfaces::srv::PowerWheelEnable>(
        	"power_wheel_enable_cmd", 
			std::bind(&TETRAPowerDriver::Power_Wheel_Enable_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerWheelEnable>("Power_wheel_enable_cmd", std::bind(&TETRAPowerDriver::Power_Wheel_Enable_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_parameter_read_srv = create_service<interfaces::srv::PowerParameterRead>(
        	"power_parameter_read_cmd", 
			std::bind(&TETRAPowerDriver::Power_Paramter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerParameterRead>("Power_parameter_read_cmd", std::bind(&TETRAPowerDriver::Power_Paramter_Read_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_parameter_write_srv = create_service<interfaces::srv::PowerParameterWrite>(
        	"power_parameter_write_cmd", 
			std::bind(&TETRAPowerDriver::Power_Paramter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerParameterWrite>("Power_parameter_write_cmd", std::bind(&TETRAPowerDriver::Power_Paramter_Write_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_data_read_srv = create_service<interfaces::srv::PowerDataRead>(
        	"power_data_read_cmd", 
			std::bind(&TETRAPowerDriver::Power_Data_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerDataRead>("Power_data_read_cmd", std::bind(&TETRAPowerDriver::Power_Data_Read_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_adc_read_srv = create_service<interfaces::srv::PowerAdcRead>(
        	"power_adc_read_cmd", 
			std::bind(&TETRAPowerDriver::Power_ADC_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerAdcRead>("Power_adc_read_cmd", std::bind(&TETRAPowerDriver::Power_ADC_Read_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_version_read_srv = create_service<interfaces::srv::PowerVersionRead>(
        	"power_version_read_cmd", 
			std::bind(&TETRAPowerDriver::Power_Version_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerVersionRead>("Power_version_read_cmd", std::bind(&TETRAPowerDriver::Power_Version_Read_Command, this, std::placeholders::_1, std::placeholders::_2)));

		conveyor_auto_movement_srv = create_service<interfaces::srv::ConveyorAutoMovement>(
        	"conveyor_auto_move_cmd", 
			std::bind(&TETRAPowerDriver::Conveyor_Auto_Movement_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::ConveyorAutoMovement>("Conveyor_auto_move_cmd", std::bind(&TETRAPowerDriver::Conveyor_Auto_Movement_Command, this, std::placeholders::_1, std::placeholders::_2)));

		conveyor_manual_movement_srv = create_service<interfaces::srv::ConveyorManualMovement>(
        	"conveyor_manual_move_cmd", 
			std::bind(&TETRAPowerDriver::Conveyor_Manual_Movement_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::ConveyorManualMovement>("Conveyor_manual_move_cmd", std::bind(&TETRAPowerDriver::Conveyor_Manual_Movement_Command, this, std::placeholders::_1, std::placeholders::_2)));

		conveyor_parameter_read_srv = create_service<interfaces::srv::ConveyorParameterRead>(
        	"conveyor_parameter_read_cmd", 
			std::bind(&TETRAPowerDriver::Conveyor_Parameter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::ConveyorParameterRead>("Conveyor_parameter_read_cmd", std::bind(&TETRAPowerDriver::Conveyor_Parameter_Read_Command, this, std::placeholders::_1, std::placeholders::_2)));

		conveyor_parameter_write_srv = create_service<interfaces::srv::ConveyorParameterWrite>(
        	"conveyor_parameter_write_cmd", 
			std::bind(&TETRAPowerDriver::Conveyor_Parameter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::ConveyorParameterWrite>("Conveyor_parameter_write_cmd", std::bind(&TETRAPowerDriver::Conveyor_Parameter_Write_Command, this, std::placeholders::_1, std::placeholders::_2)));

		conveyor_data_read_srv = create_service<interfaces::srv::ConveyorDataRead>(
        	"conveyor_data_read_cmd", 
			std::bind(&TETRAPowerDriver::Conveyor_Data_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::ConveyorDataRead>("Conveyor_data_read_cmd", std::bind(&TETRAPowerDriver::Conveyor_Data_Read_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_sonar_read_srv = create_service<interfaces::srv::PowerSonarRead>(
        	"power_sonar_read_cmd", 
			std::bind(&TETRAPowerDriver::Power_Sonar_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerSonarRead>("Power_sonar_read_cmd", std::bind(&TETRAPowerDriver::Power_Sonar_Read_Command, this, std::placeholders::_1, std::placeholders::_2)));

		power_sonar_cmd_srv = create_service<interfaces::srv::PowerSonarCmd>(
        	"power_sonar_start_cmd", 
			std::bind(&TETRAPowerDriver::Power_Sonar_Command, this, std::placeholders::_1, std::placeholders::_2));
		legacy_services_.push_back(create_service<interfaces::srv::PowerSonarCmd>("Power_sonar_start_cmd", std::bind(&TETRAPowerDriver::Power_Sonar_Command, this, std::placeholders::_1, std::placeholders::_2)));

		// 관리자 "보드 명령 테스트". 노드 이름 아래(/tetra_power_driver/raw_command)에 둬 구동 보드 것과 구분한다
		raw_command_srv = create_service<interfaces::srv::BoardRawCommand>(
			"~/raw_command",
			std::bind(&TETRAPowerDriver::Raw_Command, this, std::placeholders::_1, std::placeholders::_2));
	}

	rclcpp::Time current_time, last_time;

	// Publishers
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr power_status_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tetra_battery_publisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_voltage_publisher;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_current_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr docking_status_publisher;
	rclcpp::Publisher<interfaces::msg::Gpio>::SharedPtr gpio_status_publisher;

	// Ultrasonic Option Publishers
	std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, kSonarCount> ultrasonic_publishers;

	// Ultrasonic PointCloud2 Publishers
	std::array<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr, kSonarCount> points_publishers;

	// Conveyor Option Publishers
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr conveyor_sensor_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr conveyor_movement_publisher;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lift_status_publisher;

	// Pointcloud publisher without PCL
	void RangeToCloud(const sensor_msgs::msg::Range::SharedPtr range_msg, const std::string& frame_id, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
	{
		sensor_msgs::msg::PointCloud2 output;
		output.header = range_msg->header;
		output.header.frame_id = frame_id;
		output.height = 1;
		output.is_bigendian = false;
		output.is_dense = true;

		if (range_msg->range < m_dUltrasonicMaxRange && range_msg->range > Ultrasonic_MIN_range)
		{
			output.width = 1;
			sensor_msgs::PointCloud2Modifier modifier(output);
			modifier.setPointCloud2FieldsByString(1, "xyz");

			sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
			sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
			sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
			*iter_x = range_msg->range;
			*iter_y = 0.0f;
			*iter_z = 0.0f;
		}
		else
		{
			output.width = 0;
			sensor_msgs::PointCloud2Modifier modifier(output);
			modifier.setPointCloud2FieldsByString(1, "xyz");
		}
		pub->publish(output);
	}


	// Custom Service Callback Functions
	bool LED_Control_Command(const std::shared_ptr<interfaces::srv::LedControl::Request> request, const std::shared_ptr<interfaces::srv::LedControl::Response> response)
	{
		const int ret = dssp_rs232_power_module_set_light(request->id, request->led_brightness);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool LED_Toggle_Command(const std::shared_ptr<interfaces::srv::LedToggleControl::Request> request, const std::shared_ptr<interfaces::srv::LedToggleControl::Response> response)
	{
		const int ret = dssp_rs232_power_module_set_light_toggle(
			request->de_index, request->light_accel, request->led_high_brightness, request->light_decel, request->led_low_brightness);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Toggle_On_Command(const std::shared_ptr<interfaces::srv::ToggleOn::Request> request, const std::shared_ptr<interfaces::srv::ToggleOn::Response> response)
	{
		const int ret = dssp_rs232_power_module_toggle_on(request->id);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Integrallog_Command(const std::shared_ptr<interfaces::srv::Integrallog::Request> request, const std::shared_ptr<interfaces::srv::Integrallog::Response> response)
	{
		bool bResult = false;
		time_t timer = time(NULL); 
		struct tm* t = localtime(&timer); 

		int m_iVlotage_data[500] = {0, };
		int m_iCurrent_data[500] = {0, };
		string m_colon = ":";
		string m_time_stemp = to_string(t->tm_hour) + m_colon + to_string(t->tm_min) + m_colon + to_string(t->tm_sec);
		string m_temp1 = m_time_stemp + "_V_log";
		string m_temp2 = m_time_stemp + "_I_log";
		
		if(request->value_index == 1)
		{
			string m_strFilePathName = "/home/tetra/LOG/" + m_temp1 + ".txt";  
			dssp_rs232_power_module_read_Voltage(m_iVlotage_data); 

			fp = fopen(m_strFilePathName.c_str(), "w");
			if(fp != NULL) 
			{
				for(int i=0; i<500; i++)
				{
					fprintf(fp, "%.2f \n", (double)m_iVlotage_data[i] / 10.0);
				}
				fclose(fp);
			}
		}
		else if(request->value_index == 2)
		{
			string m_strFilePathName = "/home/tetra/LOG/" + m_temp2 + ".txt";  
			dssp_rs232_power_module_read_Current(m_iCurrent_data);

			fp = fopen(m_strFilePathName.c_str(), "w");
			if(fp != NULL) 
			{
				for(int j=0; j<500; j++)
				{
					fprintf(fp, "%.2f \n", (double)m_iCurrent_data[j] / 10.0);
				}
				fclose(fp);
			}
		}

		bResult = true;
		response->command_result = bResult;
		return true;
	}

	bool Power_Set_Outport_Command(const std::shared_ptr<interfaces::srv::PowerSetOutport::Request> request, const std::shared_ptr<interfaces::srv::PowerSetOutport::Response> response)
	{
		const int ret = dssp_rs232_power_module_set_OutputPort(
			request->output0, request->output1, request->output2, request->output3,
			request->output4, request->output5, request->output6, request->output7);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Set_Single_Outport_Command(const std::shared_ptr<interfaces::srv::PowerSetSingleOutport::Request> request, const std::shared_ptr<interfaces::srv::PowerSetSingleOutport::Response> response)
	{
		const int ret = dssp_rs232_power_module_set_Single_OutputPort(request->id, request->value);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Get_IO_Status_Command(const std::shared_ptr<interfaces::srv::PowerGetIoStatus::Request> request, const std::shared_ptr<interfaces::srv::PowerGetIoStatus::Response> response)
	{
		(void)request;
		response->input0 = gpio_msg.input0;
		response->input1 = gpio_msg.input1;
		response->input2 = gpio_msg.input2;
		response->input3 = gpio_msg.input3;
		response->input4 = gpio_msg.input4;
		response->input5 = gpio_msg.input5;
		response->input6 = gpio_msg.input6;
		response->input7 = gpio_msg.input7;

		response->output0 = gpio_msg.output0;
		response->output1 = gpio_msg.output1;
		response->output2 = gpio_msg.output2;
		response->output3 = gpio_msg.output3;
		response->output4 = gpio_msg.output4;
		response->output5 = gpio_msg.output5;
		response->output6 = gpio_msg.output6;
		response->output7 = gpio_msg.output7;

		response->command_result = true;
		return true;
	}

	bool Power_Set_Enable_Command(const std::shared_ptr<interfaces::srv::PowerSetEnable::Request> request, const std::shared_ptr<interfaces::srv::PowerSetEnable::Response> response)
	{
		const int ret = dssp_rs232_power_module_set_Enable(
			request->power0, request->power1, request->power2, request->power3,
			request->power4, request->power5, request->power6, request->power7);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Set_Single_Enable_Command(const std::shared_ptr<interfaces::srv::PowerSetSingleEnable::Request> request, const std::shared_ptr<interfaces::srv::PowerSetSingleEnable::Response> response)
	{
		const int ret = dssp_rs232_power_module_set_Single_Enable(request->id, request->value);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Wheel_Enable_Command(const std::shared_ptr<interfaces::srv::PowerWheelEnable::Request> request, const std::shared_ptr<interfaces::srv::PowerWheelEnable::Response> response)
	{
		const int ret = dssp_rs232_power_module_wheel_enable(request->on);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Paramter_Read_Command(const std::shared_ptr<interfaces::srv::PowerParameterRead::Request> request, const std::shared_ptr<interfaces::srv::PowerParameterRead::Response> response)
	{
		(void)request;
		const int ret = dssp_rs232_power_module_parameter_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,
			&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20,
			&m_iParam21,&m_iParam22,&m_iParam23,&m_iParam24,&m_iParam25,&m_iParam26,&m_iParam27,&m_iParam28,&m_iParam29,&m_iParam30,&m_iParam31);
		
		response->p0_uart1_baudrate = m_iParam0;
		response->p1_uart2_baudrate = m_iParam1;
		response->p2_sonar_select = m_iParam2;
		response->p3_sonar_distance_offset = m_iParam3;
		response->p4_sonar_quantity = m_iParam4;
		response->p5_sonar_max_distance = m_iParam5;
		response->p6_nc = m_iParam6;
		response->p7_sonar_none_detect_mode = m_iParam7;
		response->p8_battery_charging_offest = m_iParam8;
		response->p9_battery_charging_gain = m_iParam9;
		response->p10_status_led_min = m_iParam10;
		response->p11_status_led_max = m_iParam11;
		response->p12_battery_led_min = m_iParam12;
		response->p13_battery_led_max = m_iParam13;
		response->p14_led_ch1_offest = m_iParam14;
		response->p15_led_ch2_offest = m_iParam15;
		response->p16_led_ch3_offest = m_iParam16;
		response->p17_led_ch4_offest = m_iParam17;
		response->p18_led_ch5_offest = m_iParam18;
		response->p19_led_ch6_offest = m_iParam19;
		response->p20_led_ch7_offest = m_iParam20;
		response->p21_led_ch8_offest = m_iParam21;
		response->p22_power_enable = m_iParam22;
		response->p23_outport = m_iParam23;
		response->p24_battery_recharge_voltage = m_iParam24;
		response->p25_battery_recharge_offset = m_iParam25;
		response->p26_battery_min = m_iParam26;
		response->p27_battery_max = m_iParam27;
		response->p28_battery_sampling_time = m_iParam28;
		response->p29_conveyor_mode = m_iParam29;
		response->p30_conveyor_terminal_base = m_iParam30;
		response->p31_conveyor_terminal_offset = m_iParam31;
	
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Paramter_Write_Command(const std::shared_ptr<interfaces::srv::PowerParameterWrite::Request> request, const std::shared_ptr<interfaces::srv::PowerParameterWrite::Response> response)
	{
		const int ret = dssp_rs232_power_module_parameter_write(request->num, request->data);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Data_Read_Command(const std::shared_ptr<interfaces::srv::PowerDataRead::Request> request, const std::shared_ptr<interfaces::srv::PowerDataRead::Response> response)
	{
		(void)request;
		const int ret = dssp_rs232_power_module_data_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11);

		response->d0_battery_voltage = m_iParam0;
		response->d1_system_current = m_iParam1;
		response->d2_charge_current = m_iParam2;
		response->d3_charge_signal = m_iParam3;
		response->d4_inport_status = m_iParam4;
		response->d5_outport_status = m_iParam5;
		response->d6_power_status = m_iParam6;
		response->d7_charge_terminal_status = m_iParam7;
		response->d8_temperature0 = m_iParam8;
		response->d9_temperature1 = m_iParam9;
		response->d10_mobd_inport_status = m_iParam10;
		response->d11_mobd_outport_status = m_iParam11;

		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_ADC_Read_Command(const std::shared_ptr<interfaces::srv::PowerAdcRead::Request> request, const std::shared_ptr<interfaces::srv::PowerAdcRead::Response> response)
	{
		(void)request;
		const int ret = dssp_rs232_power_module_adc_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);

		response->d0_adc0 = m_iParam0;
		response->d1_adc1 = m_iParam1;
		response->d2_adc2 = m_iParam2;
		response->d3_adc3 = m_iParam3;
		response->d4_adc4 = m_iParam4;
		response->d5_adc5 = m_iParam5;
		response->d6_adc6 = m_iParam6;
		response->d7_adc7 = m_iParam7;

		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Version_Read_Command(const std::shared_ptr<interfaces::srv::PowerVersionRead::Request> request, const std::shared_ptr<interfaces::srv::PowerVersionRead::Response> response)
	{
		(void)request;
		int iResult = dssp_rs232_power_module_version_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7);

		response->d0_board_type = m_iParam0;
		response->d1_year2 = m_iParam1;
		response->d2_year1 = m_iParam2;
		response->d3_month2 = m_iParam3;
		response->d4_month1 = m_iParam4;
		response->d5_day2 = m_iParam5;
		response->d6_day1 = m_iParam6;
		response->d7_ver = m_iParam7;

		response->command_result = (iResult == 0);
		return true;
	}

	bool Conveyor_Auto_Movement_Command(const std::shared_ptr<interfaces::srv::ConveyorAutoMovement::Request> request, const std::shared_ptr<interfaces::srv::ConveyorAutoMovement::Response> response)
	{
		const int ret = dssp_rs232_power_module_conveyor_movement(request->start);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Conveyor_Manual_Movement_Command(const std::shared_ptr<interfaces::srv::ConveyorManualMovement::Request> request, const std::shared_ptr<interfaces::srv::ConveyorManualMovement::Response> response)
	{
		const int ret = dssp_rs232_power_module_conveyor_manual_movement(request->mode);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Conveyor_Parameter_Read_Command(const std::shared_ptr<interfaces::srv::ConveyorParameterRead::Request> request, const std::shared_ptr<interfaces::srv::ConveyorParameterRead::Response> response)
	{
		(void)request;
		// 응답이 없거나 패킷 에러면 컨베이어/리프트 보드가 없는 것이다. API 는 이 결과로 미장착 여부를 판단한다
		int iResult = dssp_rs232_conveyor_module_parameter_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,
			&m_iParam11,&m_iParam12,&m_iParam13,&m_iParam14,&m_iParam15,&m_iParam16,&m_iParam17,&m_iParam18,&m_iParam19,&m_iParam20);
		
		response->p0_uart1_baudrate = m_iParam0;
		response->p1_uart2_baudrate = m_iParam1;
		response->p2_mode_select = m_iParam2;
		response->p3_outport = m_iParam3;
		response->p4_nc = m_iParam4;
		response->p5_nc = m_iParam5;
		response->p6_nc = m_iParam6;
		response->p7_conveyor_dir = m_iParam7;
		response->p8_conveyor_timeout = m_iParam8;
		response->p9_fix_loading_start_delay = m_iParam9;
		response->p10_fix_loading_end_delay = m_iParam10;
		response->p11_fix_unloading_start_delay = m_iParam11;
		response->p12_fix_unloading_motor_delay = m_iParam12;
		response->p13_fix_unloading_end_delay = m_iParam13;
		response->p14_loading_start_delay = m_iParam14;
		response->p15_loading_end_delay = m_iParam15;
		response->p16_unloading_start_delay = m_iParam16;
		response->p17_unloading_end_delay = m_iParam17;
		response->p18_express_quantity = m_iParam18;
		response->p19_terminal_loading_base = m_iParam19;
		response->p20_terminal_unloading_base = m_iParam20;
	
		response->command_result = (iResult == 0);
		return true;
	}

	bool Conveyor_Parameter_Write_Command(const std::shared_ptr<interfaces::srv::ConveyorParameterWrite::Request> request, const std::shared_ptr<interfaces::srv::ConveyorParameterWrite::Response> response)
	{
		const int ret = dssp_rs232_conveyor_module_parameter_write(request->num, request->data);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Conveyor_Data_Read_Command(const std::shared_ptr<interfaces::srv::ConveyorDataRead::Request> request, const std::shared_ptr<interfaces::srv::ConveyorDataRead::Response> response)
	{
		(void)request;
		const int ret = dssp_rs232_conveyor_module_data_read(
			&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5,&m_iParam6,
			&m_iParam7,&m_iParam8,&m_iParam9,&m_iParam10,&m_iParam11,&m_iParam12,&m_iParam13);
	
		response->d0_docking_signal0 = m_iParam0;
		response->d1_docking_signal1 = m_iParam1;
		response->d2_charge_current = m_iParam2;
		response->d3_charge_signal = m_iParam3;
		response->d4_charge_voltage = m_iParam4;
		response->d5_nc = m_iParam5;
		response->d6_nc = m_iParam6;
		response->d7_ad_in1 = m_iParam7;
		response->d8_ad_in2 = m_iParam8;
		response->d9_ad_in3 = m_iParam9;
		response->d10_in_status = m_iParam10;
		response->d11_out_status = m_iParam11;
		response->d12_photo_status = m_iParam12;
		response->d13_conveyor_status = m_iParam13;
	
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Sonar_Read_Command(const std::shared_ptr<interfaces::srv::PowerSonarRead::Request> request, const std::shared_ptr<interfaces::srv::PowerSonarRead::Response> response)
	{
		(void)request;
		const int ret = dssp_rs232_power_module_sonar_read(&m_iParam0,&m_iParam1,&m_iParam2,&m_iParam3,&m_iParam4,&m_iParam5);

		response->d0_sonar0 = m_iParam0;
		response->d1_sonar1 = m_iParam1;
		response->d2_sonar2 = m_iParam2;
		response->d3_sonar3 = m_iParam3;
		response->d4_sonar4 = m_iParam4;
		response->d5_sonar5 = m_iParam5;
		
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	bool Power_Sonar_Command(const std::shared_ptr<interfaces::srv::PowerSonarCmd::Request> request, const std::shared_ptr<interfaces::srv::PowerSonarCmd::Response> response)
	{
		const int ret = dssp_rs232_power_module_set_Ultrasonic(request->start);
		// 보드가 실패를 돌려주면 그대로 알린다. 예전에는 항상 성공으로 답해 API 가 0 값을 실제 측정값으로 저장했다
		response->command_result = (ret == 0);
		return true;
	}

	// spin_some 으로 폴링 루프 안에서 불리므로 배터리·GPIO 폴링 패킷과 섞이지 않는다 (tetra_serial/raw_command_service.hpp)
	void Raw_Command(const std::shared_ptr<interfaces::srv::BoardRawCommand::Request> request, const std::shared_ptr<interfaces::srv::BoardRawCommand::Response> response)
	{
		tetra_serial::HandleRawCommand(*request, *response, dssp_rs232_power_module_raw_exchange);
		RCLCPP_WARN(this->get_logger(), "raw_command: %zu bytes sent, %zu bytes received, %s",
			response->sent.size(), response->reply.size(), response->message.c_str());
	}

private:
	rclcpp::Service<interfaces::srv::LedControl>::SharedPtr led_control_srv;
	rclcpp::Service<interfaces::srv::LedToggleControl>::SharedPtr led_toggle_control_srv;
	// 예전 대문자 서비스 이름 (Power_outport_cmd 등). 다른 PC·예전 클라이언트가 옮겨 갈 때까지 둔다
	std::vector<rclcpp::ServiceBase::SharedPtr> legacy_services_;
	rclcpp::Service<interfaces::srv::ToggleOn>::SharedPtr toggle_on_srv;
	rclcpp::Service<interfaces::srv::Integrallog>::SharedPtr integrallog_srv;
	rclcpp::Service<interfaces::srv::PowerSetOutport>::SharedPtr power_set_outport_srv;
	rclcpp::Service<interfaces::srv::PowerSetSingleOutport>::SharedPtr power_set_single_outport_srv;
	rclcpp::Service<interfaces::srv::PowerGetIoStatus>::SharedPtr power_get_io_status_srv;
	rclcpp::Service<interfaces::srv::PowerSetEnable>::SharedPtr power_set_enable_srv;
	rclcpp::Service<interfaces::srv::PowerSetSingleEnable>::SharedPtr power_set_single_enable_srv;
	rclcpp::Service<interfaces::srv::PowerWheelEnable>::SharedPtr power_wheel_enable_srv;
	rclcpp::Service<interfaces::srv::PowerParameterRead>::SharedPtr power_parameter_read_srv;
	rclcpp::Service<interfaces::srv::PowerParameterWrite>::SharedPtr power_parameter_write_srv;
	rclcpp::Service<interfaces::srv::PowerDataRead>::SharedPtr power_data_read_srv;
	rclcpp::Service<interfaces::srv::PowerAdcRead>::SharedPtr power_adc_read_srv;
	rclcpp::Service<interfaces::srv::PowerVersionRead>::SharedPtr power_version_read_srv;
	rclcpp::Service<interfaces::srv::ConveyorAutoMovement>::SharedPtr conveyor_auto_movement_srv;
	rclcpp::Service<interfaces::srv::ConveyorManualMovement>::SharedPtr conveyor_manual_movement_srv;
	rclcpp::Service<interfaces::srv::ConveyorParameterRead>::SharedPtr conveyor_parameter_read_srv;
	rclcpp::Service<interfaces::srv::ConveyorParameterWrite>::SharedPtr conveyor_parameter_write_srv;
	rclcpp::Service<interfaces::srv::ConveyorDataRead>::SharedPtr conveyor_data_read_srv;
	rclcpp::Service<interfaces::srv::PowerSonarRead>::SharedPtr power_sonar_read_srv;
	rclcpp::Service<interfaces::srv::PowerSonarCmd>::SharedPtr power_sonar_cmd_srv;
	rclcpp::Service<interfaces::srv::BoardRawCommand>::SharedPtr raw_command_srv;
};

// 전원 보드를 이만큼 연속으로 읽지 못하면 끊긴 것으로 본다 (한 번 읽는 데 최대 0.2초라 약 0.7초).
// 예전에는 10번 넘게 실패해야(약 2.5초) 서보를 껐다
constexpr int kPowerReadFailuresBeforeStop = 3;
// 구동 드라이버(power_statusCallback)가 이 값을 받으면 서보를 끈다
constexpr int kPowerBoardNoResponse = -10;
constexpr double kReconnectIntervalSec = 2.0;

// 포트를 연 직후와 다시 연 뒤에 보드에 보내는 초기 명령
static void InitPowerBoard()
{
	// Charge port Enable
	dssp_rs232_power_module_set_charging_ready(1);
	if(m_bUltrasonic_option) dssp_rs232_power_module_set_Ultrasonic(1);
}

// 전원 보드 포트를 연다. 실패하면 false (노드를 끝낸다)
static bool OpenPowerPort()
{
	if(port[0] == '\0')
	{
		sprintf(port, "/dev/ttyS1");
	}

	// RS232 Connect
	if(dssp_rs232_power_module_create(port, 200) == 0)
	{
		printf("TETRA_Power_rs232 Port (%s) Open Success\n", port);
		return true;
	}
	printf("TETRA_Power_rs232 Port (%s) Open Error!\n", port);
	return false;
}

// 초음파 Range 메시지에서 바뀌지 않는 값. 예전에는 매 주기 다시 넣었다. ultrasonic_max_range 는 노드 생성자가 읽으므로 그 뒤에 부른다
static void InitSonarMessages()
{
	for(size_t sonar = 0; sonar < kSonarCount; sonar++)
	{
		range_msgs[sonar].header.frame_id = kSonarChannels[sonar].frame_id;
		range_msgs[sonar].radiation_type = 0;
		range_msgs[sonar].field_of_view = (60.0/180.0) * M_PI;
		range_msgs[sonar].min_range = Ultrasonic_MIN_range;
		range_msgs[sonar].max_range = m_dUltrasonicMaxRange;
	}
}

// 보드를 읽지 못한 주기: 끊김을 한 번 알리고, 끊겨 있는 동안 서보를 끄게 하며 포트를 다시 연다
static void HandleBoardReadFailure(TETRAPowerDriver& node, std::chrono::steady_clock::time_point& last_reconnect_attempt)
{
	m_iFlag_PowerCheck_cnt++;
	if(m_iFlag_PowerCheck_cnt == 1)
	{
		printf("!!!! Power Board data read Error(m_iPowerCheck: %d) !!!! \n", m_iPowerCheck);
	}
	if(m_iFlag_PowerCheck_cnt == kPowerReadFailuresBeforeStop)
	{
		// 한 번만 알리고 파일로 남긴다. 예전에는 끊겨 있는 동안 매 주기 로그 파일을 새로 만들었다
		printf("[Error]: Power Board Disconnect !!! \n");
		Error_Log_write("Power Board Disconnect !!!");
		// 예전에는 실패하면 continue 로 넘어가 이 코드를 발행하지 않아, 구동 드라이버의 전원 에러 정지가 동작하지 않았다
		std_msgs::msg::Int32 power_status;
		power_status.data = kPowerBoardNoResponse;
		node.power_status_publisher->publish(power_status);
	}
	if(m_iFlag_PowerCheck_cnt >= kPowerReadFailuresBeforeStop)
	{
		std_msgs::msg::Int32 servo;
		servo.data = 2;
		node.servo_publisher->publish(servo);

		// USB 를 뺐다 꽂으면 예전 fd 로는 계속 실패하므로 포트를 새로 연다
		if(std::chrono::duration<double>(std::chrono::steady_clock::now() - last_reconnect_attempt).count() >= kReconnectIntervalSec)
		{
			last_reconnect_attempt = std::chrono::steady_clock::now();
			if(dssp_rs232_power_module_create(port, 200) == 0) InitPowerBoard();
		}
	}
}

// 보드 상태, 배터리, 도킹 접점
static void PublishPowerStatus(TETRAPowerDriver& node)
{
	// Power Board Status Check
	std_msgs::msg::Int32 power_status;
	power_status.data = m_iPowerCheck;
	node.power_status_publisher->publish(power_status);

	// Battery Level Check
	std_msgs::msg::Int32 battery_level;
	m_ibattery_Level = (int)m_dbattery;
	battery_level.data = m_ibattery_Level;
	node.tetra_battery_publisher->publish(battery_level);

	// Battery Voltage Check
	std_msgs::msg::Float64 battery_voltage;
	battery_voltage.data = m_dVoltage;
	node.battery_voltage_publisher->publish(battery_voltage);

	// Battery Current Check
	std_msgs::msg::Float64 battery_current;
	battery_current.data = m_dCurrent;
	node.battery_current_publisher->publish(battery_current);

	// Docking Status Check
	std_msgs::msg::Int32 docking_status;
	docking_status.data = m_imode_status;
	node.docking_status_publisher->publish(docking_status);
}

static void PublishGpio(TETRAPowerDriver& node)
{
	// GPIO Check
	for(size_t gpio_port = 0; gpio_port < kGpioPortCount; gpio_port++)
	{
		gpio_msg.*kGpioInputFields[gpio_port] = m_iInput[gpio_port];
		gpio_msg.*kGpioOutputFields[gpio_port] = m_iOutput[gpio_port];
	}
	node.gpio_status_publisher->publish(gpio_msg);
}

static void PublishSonar(TETRAPowerDriver& node)
{
	// Ultrasonic data Check Option
	for(size_t sonar = 0; sonar < kSonarCount; sonar++)
	{
		range_msgs[sonar].range = UltrasonicRangeOrMax(m_dUltrasonic[sonar]);
		range_msgs[sonar].header.stamp = node.current_time;
	}

	if(m_bUltrasonic_option)
	{
		for(size_t sonar = 0; sonar < kSonarCount; sonar++)
		{
			node.ultrasonic_publishers[sonar]->publish(range_msgs[sonar]);
		}
		for(size_t sonar = 0; sonar < kSonarCount; sonar++)
		{
			node.RangeToCloud(std::make_shared<sensor_msgs::msg::Range>(range_msgs[sonar]),
				kSonarChannels[sonar].frame_id, node.points_publishers[sonar]);
		}
	}
}

static void PublishConveyor(TETRAPowerDriver& node)
{
	// Conveyor Option Check
	if(m_bConveyor_option)
	{
		// 읽지 못한 값을 내보내면 API 가 적재 상태를 0(빈 상태)으로 바꾼다
		std_msgs::msg::Int32 conveyor_sensor;
		if(dssp_rs232_power_module_read_conveyor_sensor(&m_dConveyor_sensor) == 0)
		{
			conveyor_sensor.data = m_dConveyor_sensor;
			node.conveyor_sensor_publisher->publish(conveyor_sensor);
		}

		std_msgs::msg::Int32 conveyor_movement;
		if(dssp_rs232_power_module_read_conveyor_movement(&m_iConveyor_movement) == 0)
		{
			conveyor_movement.data = m_iConveyor_movement;
			node.conveyor_movement_publisher->publish(conveyor_movement);
		}
	}
}

static void PublishLiftPosition(TETRAPowerDriver& node)
{
	if(!m_bLift_position_option) return;
	if(++m_iLift_poll_count < kLiftPollEvery) return;
	m_iLift_poll_count = 0;
	// 읽지 못한 값(0)을 내면 가장 높은 자리로 보이므로 실패하면 내지 않는다
	if(dssp_rs232_power_module_read_motor_position(&m_iLift_position) != 0) return;
	std_msgs::msg::Int32 lift_status;
	lift_status.data = m_iLift_position;
	node.lift_status_publisher->publish(lift_status);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	std::signal(SIGINT, signal_handler);
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	auto node = std::make_shared<TETRAPowerDriver>();

	rclcpp::WallRate loop_rate(30);

	if(!OpenPowerPort())
	{
		return -1;
	}

	InitPowerBoard();
	InitSonarMessages();
	auto last_reconnect_attempt = std::chrono::steady_clock::now();

	printf("=========================================\n");
	printf(" TETRA Power Driver Node Started (Port: %s)\n", port);
	printf("=========================================\n");

	while (rclcpp::ok() && !stop_requested)
	{
		rclcpp::spin_some(node);

		node->current_time = node->now();
    	node->last_time = node->now();

		// Power Status Check Loop (error check)
		m_iPowerCheck = dssp_rs232_power_module_read_tetra(&m_dbattery, &m_dVoltage, &m_dCurrent, &m_imode_status, m_iInput, m_iOutput, m_dUltrasonic);
		if(m_iPowerCheck < 0)
		{
			HandleBoardReadFailure(*node, last_reconnect_attempt);
			loop_rate.sleep();
			continue;
		}
		if(m_iFlag_PowerCheck_cnt >= kPowerReadFailuresBeforeStop)
		{
			printf("Power Board connection restored\n");
		}
		m_iFlag_PowerCheck_cnt = 0;

		PublishPowerStatus(*node);
		PublishGpio(*node);
		PublishSonar(*node);
		PublishConveyor(*node);
		PublishLiftPosition(*node);

		loop_rate.sleep();
    }

	// Ultrasonic Off
	dssp_rs232_power_module_set_Ultrasonic(0);
	// RS232 Disconnect
	dssp_rs232_power_module_destroy();

	rclcpp::shutdown();
    return 0;
}
