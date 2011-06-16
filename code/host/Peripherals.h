#pragma once

//
// Constants
//

// Debugging
#define DEBUG_SERIAL				      0

// Serial port
#define SERIALPORT					      "COM1"
#define SERIAL_BUFFER_SIZE			  1024

// RxSerial packet data
#define MOTION_SENSOR1_ACTIVATED	0x01	
#define MOTION_SENSOR2_ACTIVATED	0x02
#define DOOR_SENSOR1_ACTIVATED		0x04
#define DOOR_SENSOR2_ACTIVATED		0x08
#define WINDOW_SENSORS_ACTIVATED	0x10
#define SMOKE_SENSOR_ACTIVATED    0x20

// TxSerial packet data
#define ACTIVATE_FLOODLIGHTS		  0x01
#define DEACTIVATE_FLOODLIGHTS		0x02
#define ACTIVATE_SIRENS				    0x04
#define DEACTIVATE_SIRENS			    0x08
#define ACTIVATE_BUZZER_LED			  0x10
#define DEACTIVATE_BUZZER_LED		  0x20

// Friendly class
class CTouchScreenDlg;

class CPeripherals
{
public:
	CPeripherals(void);
	~CPeripherals(void);

// Attributes

public: 
	
	// Owner object.
	CTouchScreenDlg* m_pOwner;

	// Baudrate of serial port.
	int m_baudRate;

	// Summons death of serial thread.
	bool m_killReadSerialPortThread;

private:

	// Serial port handle.
	HANDLE m_serialPortHandle;

    // Flag used to represent state of serial port.
	bool m_serialPortOpen;

	// Stores activation/deactivation bit for each peripherals.
	unsigned char m_activateFloodlights;
	unsigned char m_deactivateFloodlights;
	unsigned char m_activateSirens;
	unsigned char m_deactivateSirens;
	unsigned char m_activateBuzzerLed;
	unsigned char m_deactivateBuzzerLed;

// Operations

public:

	// Starts the serial thread.
	bool InitPeripherals(CTouchScreenDlg* pOwner);

	// Reads data from the serial port.
	void ReadPort(unsigned char* Data, int Length, DWORD NumBytesRead);

	// Writes data to the serial port.
	void WritePort(unsigned char* Data, int Length, DWORD NumBytesWritten);

	// Close the serial port.
	bool ClosePort();

	// Activate the buzzer and led.
	bool Buzzer_LED(bool State);

	// Activate the siren.
	bool Siren(unsigned int SirenNumber, bool State);

	// Activate the flood light.
	bool FloodLight(unsigned int FloodLightNumber, bool State);

	// Activate the strobe light.
	bool StrobeLight(unsigned int StrobeLightNumber, bool State);

private: 

	// Activate the serial port.
	bool InitSerialPort(void);
};