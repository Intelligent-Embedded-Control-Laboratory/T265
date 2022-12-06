// serial_rs232.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。
//

#include <iostream>
#include "..\include\rs232\USBSerial.hpp"

#ifdef _WIN32
#define delay(ms) Sleep(ms) // Delay ms millisecond
#else
#define delay(ms) usleep(ms*1000); // Delay ms millisecond
#endif

#define USB_SERIAL_COM_PORT 3
#define USB_SERIAL_BAUD_RATE 115200
#define USB_SERIAL_FORMAT "8E1"

typedef struct
{
    float T[3];
    float V[3];
    float Q[4];
    int confidence;
} uart_tx_data_t; // define data structure for serial data communication

USBSerial Serial;
uart_tx_data_t uart_tx_data;


// Main program enter point:
int main()
{
	if (Serial.begin(USB_SERIAL_COM_PORT, USB_SERIAL_BAUD_RATE, USB_SERIAL_FORMAT) == -1)
	{
		std::cout << "Open COM " << USB_SERIAL_COM_PORT << " Failed ...\n";
		system("pause");
		return EXIT_SUCCESS;
	}
	std::cout << "Open COM" << USB_SERIAL_COM_PORT << " Successfully!" << std::endl;

	memset(&uart_tx_data, 0, sizeof(uart_tx_data_t));

	std::cout << "Data Transmitting ...\n";
	for (;;)
	{
		static int counter;
		counter++;
		
		uart_tx_data.confidence = counter;
		Serial.write_bytes((uint8_t*)&uart_tx_data, sizeof(uart_tx_data_t));

		std::cout << uart_tx_data.confidence << "\n";
		delay(1000); // Delay 1000 milliseconds for 1 Hz sampling frequency
	}
}

// 執行程式: Ctrl + F5 或 [偵錯] > [啟動但不偵錯] 功能表
// 偵錯程式: F5 或 [偵錯] > [啟動偵錯] 功能表

// 開始使用的提示: 
//   1. 使用 [方案總管] 視窗，新增/管理檔案
//   2. 使用 [Team Explorer] 視窗，連線到原始檔控制
//   3. 使用 [輸出] 視窗，參閱組建輸出與其他訊息
//   4. 使用 [錯誤清單] 視窗，檢視錯誤
//   5. 前往 [專案] > [新增項目]，建立新的程式碼檔案，或是前往 [專案] > [新增現有項目]，將現有程式碼檔案新增至專案
//   6. 之後要再次開啟此專案時，請前往 [檔案] > [開啟] > [專案]，然後選取 .sln 檔案
