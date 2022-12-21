#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <windows.h> // 設定優先權
#include "..\include\librealsense2\rs.hpp"
#include "..\samples\example-utils.hpp"
#include "..\include\rs232\USBSerial.hpp"
#include "..\include\attitude\attitude.hpp"

// USER DEFINES
#define USB_SERIAL_COM_PORT 3
#define USB_SERIAL_BAUD_RATE 115200
#define USB_SERIAL_FORMAT "8E1"

#define USER_ENTER_COM_PORT 0
#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
// ENDOF USER DEFINES

// USER TYPEDEFS
typedef struct /* T265 serial data transmission data structure */
{
	float T[3]; // Position (m)
	float V[3]; // Velocity (m/s)
	float Q[4]; // Quaternion 
	int confidence; // Confidence
} uart_data_t;
struct uart_rx_data_t
{
	uart_rx_data_t()
	{
		memset(this, 0, sizeof(uart_rx_data_t));
	}
	uint8_t value;
	int ESC_ms[4];
} uart_rx_data, uart_rx_data_pre;
// END OF USER TYPEDEFS

enum T265_STATUS : int
{
	T265_DETECT_NAN = -1,
	T265_OK
};

USBSerial Serial;
uart_data_t uart_data;
int T265_status = T265_OK;
bool fileio_trigger = false;
float euler[3]; // Euler angle (from quaternion)

// T265 Libiary
rs2::pipeline pipe; // Declare RealSense pipeline, encapsulating the actual deviceand sensors
rs2::config cfg; // Create a configuration for configuring the pipeline with a non default profile

// USER FUNCTION DEFINITIONS
void T265_reboot()
{
	pipe.stop(); // Stop the pipeline streaming.
	pipe.start(cfg); // Start the pipeline streaming according to the configuration.

	T265_status = T265_OK;
}


void print_concole()
{
	for (;;)
	{
		auto ms = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

		/* Convert T265 quaternion data to Euler angle*/
		Attitude::quaternion_to_euler(uart_data.Q, euler);
		for (auto &s : euler)
		{
			s *= RAD_TO_DEG;
		}

		/* Print Data to Concole */
		std::cout << std::setprecision(3) << std::fixed;
		std::cout
			<< uart_data.confidence << "\t"
			<< uart_data.T[0] << "\t"
			<< uart_data.T[1] << "\t"
			<< uart_data.T[2] << "\t"
			<< uart_data.V[0] << "\t"
			<< uart_data.V[1] << "\t"
			<< uart_data.V[2] << "\t"
			<< euler[0] << "\t"
			<< euler[1] << "\t"
			<< euler[2] << "\n";

		std::this_thread::sleep_until(ms);
	}
}

/* Program Enter Point: */
int main(int argc, char* argv[]) try
{
#if USER_ENTER_COM_PORT
	char num[8];
	cout << "Please Enter the COM Port Number: (Enter 1 for COM1)" << endl;
	fgets(num, 7, stdin);
	cport_nr = atoi(num) - 1; // 6
#endif
	std::thread t_print(print_concole);

	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

	// Open Serial
	if (Serial.begin(USB_SERIAL_COM_PORT, USB_SERIAL_BAUD_RATE, USB_SERIAL_FORMAT) == -1)
	{
		std::cout << "Open COM " << USB_SERIAL_COM_PORT << " Failed ...\n";
	
		system("pause");
		return EXIT_FAILURE;
	}
	std::cout << "Open COM" << USB_SERIAL_COM_PORT << " Successfully!" << std::endl;

	/* Data Initialization */
	memset(&uart_data, 0, sizeof(uart_data_t));

	// Initialize file stream
	std::string file_name = "data.bin";
	std::fstream fs; 
	fs.open("data.bin", std::ios::out | std::ios::binary);
	if (!fs.is_open())
	{
		std::cout << "Failed to open the file " + file_name + ".\n";
		
		system("pause");
		return EXIT_FAILURE;
	}
	std::cout << "Open the file " + file_name + " successfaully.\n";

	// T265 device initialization
	std::string serial;
	if (!device_with_streams({ RS2_STREAM_POSE }, serial))
		return EXIT_SUCCESS;
	if (!serial.empty())
		cfg.enable_device(serial);
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF); // Add pose stream
	pipe.start(cfg); // Start pipeline with chosen configuration

	/* Main loop: infinite loop for transmitting data by UART continuously */
	std::cout << "Data Transmitting ..." << std::endl;
	while (true)
	{
		auto frames = pipe.wait_for_frames(); // Wait for the next set of frames from the camera
		auto f = frames.first_or_default(RS2_STREAM_POSE); // Get a frame from the pose stream
		auto pose_data = f.as<rs2::pose_frame>().get_pose_data(); // Cast the frame to pose_frame and get its data

		uart_data.confidence = pose_data.tracker_confidence;

		/* Check the data correctness and align all data to front-left-up (x-y-z) coordinate */
		// Position
		if (fpclassify(pose_data.translation.z) != FP_NAN && fpclassify(pose_data.translation.x) != FP_NAN && fpclassify(pose_data.translation.y) != FP_NAN)
		{
			uart_data.T[0] = -pose_data.translation.z;
			uart_data.T[1] = -pose_data.translation.x;
			uart_data.T[2] = pose_data.translation.y;
		}
		else
		{
			uart_data.confidence = -1;
			T265_status = T265_DETECT_NAN;
			std::cout << "偵測到位置 NAN!\n";
		}
		// Velocity
		if (fpclassify(pose_data.velocity.z) != FP_NAN && fpclassify(pose_data.velocity.x) != FP_NAN && fpclassify(pose_data.velocity.y) != FP_NAN)
		{
			uart_data.V[0] = -pose_data.velocity.z;
			uart_data.V[1] = -pose_data.velocity.x;
			uart_data.V[2] = pose_data.velocity.y;
		}
		else
		{
			uart_data.confidence = -1;
			T265_status = T265_DETECT_NAN;
			std::cout << "偵測到速度 NAN!\n";
		}
		// Quaternion
		if (fpclassify(pose_data.rotation.w) != FP_NAN && fpclassify(pose_data.rotation.x) != FP_NAN
			&& fpclassify(pose_data.rotation.y) != FP_NAN && fpclassify(pose_data.rotation.z) != FP_NAN)
		{
			uart_data.Q[0] = pose_data.rotation.w;
			uart_data.Q[1] = -pose_data.rotation.z;
			uart_data.Q[2] = -pose_data.rotation.x;
			uart_data.Q[3] = pose_data.rotation.y;
		}
		else
		{
			uart_data.confidence = -1;
			T265_status = T265_DETECT_NAN;
			std::cout << "偵測到姿態 NAN!\n";
		}

		/* Send binary-encoded T265 data to UART */
		Serial.write_bytes((uint8_t*)&uart_data, sizeof(uart_data_t));

		/* Read binary-encoded data from UART (RC camera capture operation) */
		int status = Serial.read_bytes((uint8_t*)&uart_rx_data.value, sizeof(uart_rx_data_t));
		if (status == 0)
		{
			/* Do something when receiving the RX data from Teensy */
			
			// File IO trigger
			if (uart_rx_data.value - uart_rx_data_pre.value == (uint8_t)1)
			{
				fileio_trigger = !fileio_trigger;
				if (fileio_trigger)
				{
					std::cout << "Begin the write T265 data to the file " + file_name + "...\n";
				}
				else
				{
					std::cout << "Finish File IO: " + file_name + ".\n";
				}
			}
			if (fileio_trigger)
			{
				/* Write file */
				fs.write((char*)&uart_data, sizeof(uart_data_t));
			}
			else
			{
				fs.flush();
				fs.close();
			}
		}

		/* Emergence measures for T265 tracking diverging */
		if (T265_status == T265_DETECT_NAN)
		{
			std::cout << "偵測到數據 NAN, 正在停止 T265 串流並重新啟動 ...\n";

			T265_reboot();
		}
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	system("pause");

	//T265_reboot();
	//std::cout << "Reboting ...\n";

	//system("pause");

	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;

	system("pause");
	return EXIT_FAILURE;
}
