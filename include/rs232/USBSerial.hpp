/* 
    A serial communication library for PC platform using RS-232 protocol.
	2022-12-05 
*/

#pragma once

#include <iostream>
#include "rs232.h"

#define USB_SERIAL_BUF_SIZE 4096

class USBSerial
{
public:
	USBSerial()
	{
		memset(this, 0, sizeof(USBSerial));
		start_byte = 0xAA;
		finish_byte = 0xBB;
	}
	void set_start_byte(uint8_t _start_byte)
	{
		start_byte = _start_byte;
	}
	void set_finish_byte(uint8_t _finish_byte)
	{
		finish_byte = _finish_byte;
	}
	void set_com_port(int com_port)
	{
		cport_nr = com_port - 1; /* /dev/ttyS0 (COM1 on windows) */
	}
	void set_baud_rate(int _baud_rate)
	{
		baud_rate = _baud_rate;
	}
	int begin(int _com_port_number, int _baud_rate, const char* _mode)
	{
		set_com_port(_com_port_number);
		set_baud_rate(_baud_rate);

		mode[0] = _mode[0];
		mode[1] = _mode[1];
		mode[2] = _mode[2];
		mode[3] = _mode[3];

		if (RS232_OpenComport(cport_nr, baud_rate, mode, 0))
		{
			return -1;
		}
		return 0;
	}
	void write_bytes(uint8_t* _data, int _nbytes)
	{
		/* Send binary-encoded T265 data to UART */
		RS232_SendByte(cport_nr, start_byte); // Send 1 start byte
		RS232_SendBuf(cport_nr, _data, _nbytes); // Send data bytes
		RS232_SendByte(cport_nr, finish_byte); // Send 1 finish byte
	}
	int read_bytes(uint8_t* _dest, int _nbytes)
	{
		n = RS232_PollComport(cport_nr, buf, USB_SERIAL_BUF_SIZE - 1); // 獲得最近一次傳過來的資料，如果對方停止傳輸，就讀不到東西

		if (n >= _nbytes + 2)
		{
			for (i = 0; i < n; i++)
			{
				if (buf[i] == start_byte) // Check if the 'Start_Byte' is correct
				{
					if (buf[i + _nbytes + 1] == finish_byte) // Check if the 'Finish_Byte' is correct
					{
						memcpy(_dest, &buf[i + 1], _nbytes); // Copy data bytes

						return 0;
					}
					return -3;
				}
				else
				{
					return -2;
				}
			}
		}
		else
		{
			return -1;
		}
	}

protected:
	uint8_t start_byte;
	uint8_t finish_byte;
	int i, n;
	int cport_nr;
	int baud_rate;
	uint8_t buf[USB_SERIAL_BUF_SIZE];
	char mode[4];
};