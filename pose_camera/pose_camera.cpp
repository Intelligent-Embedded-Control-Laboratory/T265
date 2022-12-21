/* */
// 2022-12-05
#include <iostream>

// Threads 
#include <thread>
#include <chrono>
#include <queue>
#include <windows.h> // 設定優先權

// OpenCV
#include <opencv2/opencv.hpp>

//時間相關
#include <Windows.h> 

//創立資料夾相關
#include <io.h>
#include <string>
#include <direct.h>

// T265 Tracking Camera
#include <iomanip>
#include "../include/librealsense2/rs.hpp"
#include "../samples/example-utils.hpp"

#include "..\include\rs232\USBSerial.hpp" // Serial
#include "..\include\attitude\attitude.hpp" // Attitude kinematics library

// DEFINES
#define USB_SERIAL_COM_PORT 3
#define USB_SERIAL_BAUD_RATE 115200
#define USB_SERIAL_FORMAT "8E1"

#define USER_ENTER_COM_PORT 0
#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// USER TYPEDEFS
typedef struct /* T265 serial data transmission data structure */
{
	float T[3]; // Position (m)
	float V[3]; // Velocity (m/s)
	float Q[4]; // Quaternion 
	int confidence; // Confidence
} uart_tx_data_t;

typedef struct
{
	uint8_t value;
} uart_rx_data_t;

enum T265_STATUS : int
{
	T265_DETECT_NAN = -1,
	T265_OK
};
// END OF USER 

// Serail
USBSerial Serial;
uart_rx_data_t rx_data, rx_data_pre;
uart_tx_data_t tx_data;

// T265
int T265_status = T265_OK;

float euler[3]; // Euler angle (from quaternion)
float quat_c[4]; // Calibrated quaternion (Failed)
float euler_c[3]; // Calibrated Euler angle (from the calibrated quaternion) (Failed)

// T265 Libiary
rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the actual deviceand sensors
rs2::config cfg; // Create a configuration for configuring the pipeline with a non default profile

// OpenCV
cv::VideoCapture cap(0); // OpenCV
cv::Mat frame;
cv::Mat frame_resize;
int frame_scale = 2;
int flag; // 按鍵存取變數
char date_str[1024]; // time
SYSTEMTIME sys; // time
std::string file_name; // file name
std::string folder;


// USER FUNCTION DEFINITIONS
void T265_reboot()
{
	pipe.stop(); // Stop the pipeline streaming.
	pipe.start(cfg); // Start the pipeline streaming according to the configuration.

	T265_status = T265_OK;
}

void save_frame()
{
	memset(date_str, '/0', sizeof(date_str)); // 產生時間字串
	GetLocalTime(&sys);
	snprintf(date_str, sizeof(date_str), "%4d_%02d_%02d_%02d_%02d_%02d_%03d", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);

	file_name = folder + date_str + ".png"; // 存放的位置與名稱
	std::cout << "Save the image file in " << file_name << std::endl; // Concole 跳儲存提示
	cv::imwrite(file_name, frame); // 存 frame
}
void camera_control()
{
	// Open Camera
	if (!cap.isOpened())
	{
		std::cout << "Cannot open camera\n";
	}
	else
	{
		cap.set(cv::CAP_PROP_FRAME_WIDTH, 1080);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
	}

	// 抓取系統時間以創建資料夾
	GetLocalTime(&sys);
	snprintf(date_str, sizeof(date_str), "%4d_%02d_%02d", sys.wYear, sys.wMonth, sys.wDay);

	// 創立資料夾
	folder = "C:/Users/IEC5892M/Desktop/FPV_Capture/";
	folder += date_str;
	//folder = date_str;
	folder += "/";
	if (_access(folder.c_str(), 0) == -1) // -1 代表沒有資料夾
	{
		_mkdir(folder.c_str()); // 沒有就創建
	}

	// Loop
	for (;;)
	{
		// 擷取影像
		bool ret = cap.read(frame);
		if (!ret)
		{
			std::cout << "Cannot read the image\n";
			break;
		}
		

		// 放大 frame
		//cv::resize(frame, frame_resize, cv::Size(frame.cols * frame_scale, frame.rows * frame_scale), 0, 0, cv::INTER_LINEAR);
		cv::imshow("live", frame); // 顯示圖片

		flag = cv::waitKey(1); // 等待 1 ms
		if (flag == 'q') // 按下 q 鍵離開迴圈
		{
			break;
		}
		else if (flag == 's')
		{
			save_frame();
		}

		if (rx_data.value - rx_data_pre.value >= (uint8_t)1)
		{
			save_frame();
		}
		memcpy(&rx_data_pre, &rx_data, sizeof(uart_rx_data_t));
	}
}

void print_console()
{
	for (;;)
	{
		auto ms = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

		/* Convert T265 quaternion data to Euler angle*/
		Attitude::quaternion_to_euler(tx_data.Q, euler);
		for (int i = 0; i < 3; i++)
		{
			euler[i] *= RAD_TO_DEG; // Unit: degree
		}

		/* Print Data to Concole */
		std::cout << std::setprecision(3) << std::fixed;
		std::cout
			<< tx_data.confidence << "\t"
			<< tx_data.T[0] << "\t"
			<< tx_data.T[1] << "\t"
			<< tx_data.T[2] << "\t"
			<< tx_data.V[0] << "\t"
			<< tx_data.V[1] << "\t"
			<< tx_data.V[2] << "\t"
			<< euler[0] << "\t"
			<< euler[1] << "\t"
			<< euler[2] << "\n";

		std::this_thread::sleep_until(ms);
	}
}
// Main Program Enter Point:
int main()
{
	std::thread t_camera(camera_control);
	std::thread t_console(print_console);

	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
	
	// Open Serial
	if (Serial.begin(USB_SERIAL_COM_PORT, USB_SERIAL_BAUD_RATE, USB_SERIAL_FORMAT) == -1)
	{
		std::cout << "Open COM " << USB_SERIAL_COM_PORT << " Failed ...\n";
		system("pause");
		return EXIT_SUCCESS;
	}
	std::cout << "Open COM" << USB_SERIAL_COM_PORT << " Successfully!" << std::endl;

	// T265 device initialization
	std::string serial;
	if (!device_with_streams({ RS2_STREAM_POSE }, serial))
		return EXIT_SUCCESS;
	if (!serial.empty())
		cfg.enable_device(serial);
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF); // Add pose stream
	pipe.start(cfg); // Start pipeline with chosen configuration

	// -------------------------------------------------------
	std::cout << "Data receiving ...\n";
	while (true)
	{
		auto frames = pipe.wait_for_frames(); // Wait for the next set of frames from the camera
		auto f = frames.first_or_default(RS2_STREAM_POSE); // Get a frame from the pose stream
		auto pose_data = f.as<rs2::pose_frame>().get_pose_data(); // Cast the frame to pose_frame and get its data

		tx_data.confidence = pose_data.tracker_confidence;

		/* Check the data correctness and align all data to front-left-up (x-y-z) coordinate */
		// Position
		if (fpclassify(pose_data.translation.z) != FP_NAN && fpclassify(pose_data.translation.x) != FP_NAN && fpclassify(pose_data.translation.y) != FP_NAN)
		{
			tx_data.T[0] = -pose_data.translation.z;
			tx_data.T[1] = -pose_data.translation.x;
			tx_data.T[2] = pose_data.translation.y;
		}
		else
		{
			tx_data.confidence = -1;
			T265_status = T265_DETECT_NAN;
			std::cout << "偵測到位置 NAN!\n";
		}
		// Velocity
		if (fpclassify(pose_data.velocity.z) != FP_NAN && fpclassify(pose_data.velocity.x) != FP_NAN && fpclassify(pose_data.velocity.y) != FP_NAN)
		{
			tx_data.V[0] = -pose_data.velocity.z;
			tx_data.V[1] = -pose_data.velocity.x;
			tx_data.V[2] = pose_data.velocity.y;
		}
		else
		{
			tx_data.confidence = -1;
			T265_status = T265_DETECT_NAN;
			std::cout << "偵測到速度 NAN!\n";
		}
		// Quaternion
		if (fpclassify(pose_data.rotation.w) != FP_NAN && fpclassify(pose_data.rotation.x) != FP_NAN
			&& fpclassify(pose_data.rotation.y) != FP_NAN && fpclassify(pose_data.rotation.z) != FP_NAN)
		{
			tx_data.Q[0] = pose_data.rotation.w;
			tx_data.Q[1] = -pose_data.rotation.z;
			tx_data.Q[2] = -pose_data.rotation.x;
			tx_data.Q[3] = pose_data.rotation.y;
		}
		else
		{
			tx_data.confidence = -1;
			T265_status = T265_DETECT_NAN;
			std::cout << "偵測到姿態 NAN!\n";
		}

		/* Send binary-encoded T265 data to UART */
		Serial.write_bytes((uint8_t*)&tx_data, sizeof(uart_tx_data_t));
		
		/* Read binary-encoded data from UART (RC camera capture operation) */
		int status = Serial.read_bytes((uint8_t*)&rx_data.value, sizeof(rx_data));
		if (status == 0)
		{
			// Do something when receving serial data
		}

		// T265 Rebot Operation
		if (T265_status == T265_DETECT_NAN)
		{
			std::cout << "偵測到數據 NAN, 正在停止 T265 串流並重新啟動 ...\n";

			T265_reboot();
		}
	}
}