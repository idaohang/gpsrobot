#include "StdAfx.h"
#include "peripherals.h"
#include "MainFrm.h"
#include "TouchScreenDlg.h"

// Global Threads
UINT TestRxPeripheralDataThread(LPVOID param);
UINT RxPeripheralDataThread(LPVOID param);

// Global variables

CPeripherals::CPeripherals(void)
{
	m_serialPortHandle			    = 0;
	m_serialPortOpen			      = false;
	m_baudRate					        = 9600;
	m_killReadSerialPortThread	= false;

	m_activateFloodlights	      = 0x01;
	m_deactivateFloodlights	    = 0x02;
	m_activateSirens		        = 0x04;
	m_deactivateSirens		      = 0x08;
	m_activateBuzzerLed		      = 0x10;
	m_deactivateBuzzerLed	      = 0x20;
}

CPeripherals::~CPeripherals(void)
{
	ClosePort();
}

bool CPeripherals::InitPeripherals(CTouchScreenDlg* pOwner)
{
	m_pOwner = pOwner;

	m_killReadSerialPortThread = false;

	InitSerialPort();

	TRACE("Serial Communications Thread Started.\n");

	return true;
}

bool CPeripherals::InitSerialPort(void)
{
	if(m_serialPortOpen == false)
	{
		// Initializing port
		TRACE("Initializing Serial Port\n");
   
		DCB          PortDCB;
		COMMTIMEOUTS CommTimeouts;

		// Open the serial port.
		m_serialPortHandle = CreateFile (SERIALPORT,	// Pointer to the name of the port
							GENERIC_READ | GENERIC_WRITE,	      // Access (read/write) mode															
							0,								                  // Share mode
							NULL,							                  // Pointer to the security attribute
							OPEN_EXISTING,					            // How to open the serial port
							0,								                  // Port attributes
							NULL);							                // Handle to port with attribute to copy

		// If it fails to open the port, return FALSE.
		if ( m_serialPortHandle == INVALID_HANDLE_VALUE )
		{  
			// Could not open the port.
			return FALSE;
		}

		PortDCB.DCBlength = sizeof (DCB);     

		// Get the default port setting information.
		GetCommState (m_serialPortHandle, &PortDCB);

		// Change the DCB structure settings.
		PortDCB.BaudRate          = m_baudRate;			    // Current baud 
		PortDCB.fBinary           = TRUE;				        // Binary mode; no EOF check 
		PortDCB.fParity           = TRUE;				        // Enable parity checking. 
		PortDCB.fOutxCtsFlow      = FALSE;			        // No CTS output flow control 
		PortDCB.fOutxDsrFlow      = FALSE;			        // No DSR output flow control 
		PortDCB.fDtrControl       = DTR_CONTROL_ENABLE; // DTR flow control type 												
		PortDCB.fDsrSensitivity   = FALSE;		          // DSR sensitivity 
		PortDCB.fTXContinueOnXoff = TRUE;		            // XOFF continues Tx 
		PortDCB.fOutX             = FALSE;					    // No XON/XOFF out flow control 
		PortDCB.fInX              = FALSE;					    // No XON/XOFF in flow control 
		PortDCB.fErrorChar        = FALSE;				      // Disable error replacement. 
		PortDCB.fNull             = FALSE;					    // Disable null stripping. 
		PortDCB.fRtsControl       = RTS_CONTROL_ENABLE; // RTS flow control 												
		PortDCB.fAbortOnError     = FALSE;			        // Do not abort reads/writes on error.
		PortDCB.ByteSize          = 8;					        // Number of bits/bytes, 4-8 
		PortDCB.Parity            = NOPARITY;				    // 0-4   = no,odd,even,mark,space 
		PortDCB.StopBits          = ONESTOPBIT;			    // 0,1,2 = 1, 1.5, 2 

		// Configure the port according to the specifications of the DCB structure.
		if (!SetCommState (m_serialPortHandle, &PortDCB))
		{
			// Could not create the read thread.
			return FALSE;
		}

		// Retrieve the time-out parameters for all read and write operations on the port. 
		GetCommTimeouts (m_serialPortHandle, &CommTimeouts);

		// Change the COMMTIMEOUTS structure settings.
		CommTimeouts.ReadIntervalTimeout			    = MAXDWORD;  
		CommTimeouts.ReadTotalTimeoutMultiplier		= MAXDWORD;  
		CommTimeouts.ReadTotalTimeoutConstant		  = 200;    
		CommTimeouts.WriteTotalTimeoutMultiplier  = 10;  
		CommTimeouts.WriteTotalTimeoutConstant    = 1000;   

		// Set the time-out parameters for all read and write operations on the port. 
		if (!SetCommTimeouts (m_serialPortHandle, &CommTimeouts))
		{
			// Unable to set the time-out parameters
			return FALSE;
		}

		// Direct the port to perform extended functions SETDTR and SETRTS.
		// SETDTR: Sends the DTR (data-terminal-ready) signal.
		// SETRTS: Sends the RTS (request-to-send) signal. 
		EscapeCommFunction(m_serialPortHandle, SETDTR);
		EscapeCommFunction(m_serialPortHandle, SETRTS);
	}

	// Create a read thread for reading data from the communication port.
	#if DEBUG_SERIAL
		AfxBeginThread(TestRxPeripheralDataThread, m_pOwner, THREAD_PRIORITY_TIME_CRITICAL);
	#else 
		AfxBeginThread(RxPeripheralDataThread, m_pOwner, THREAD_PRIORITY_TIME_CRITICAL);
	#endif

	m_serialPortOpen = true;

	return TRUE;
}

void CPeripherals::WritePort(unsigned char* Data, int Length, DWORD NumBytesWritten)
{
	if(!WriteFile(m_serialPortHandle, Data, Length, &NumBytesWritten, NULL))								
	{
		TRACE("\nwritePort Failed!");
	}
	
	TRACE("\nWrote %d bytes to the serial port", NumBytesWritten);
}

void CPeripherals::ReadPort(unsigned char* Data, int Length, DWORD NumBytesRead)
{
	DWORD NumBytes = 0;

	if(!ReadFile(m_serialPortHandle, Data, Length, &NumBytes, NULL))								
	{
		TRACE("\nreadPort Failed!");
	}
	
	TRACE("\nRead %d bytes from the serial port.\n", NumBytes);
}

bool CPeripherals::ClosePort()
{	
	WSACleanup();

	m_killReadSerialPortThread = true;

	// Give thread time to finish and exit.
	Sleep(100);

	if(m_serialPortOpen)
	{
		CloseHandle(m_serialPortHandle);
	}

	m_serialPortOpen = false;

	return true;
}

bool CPeripherals::Buzzer_LED(bool State)
{
	DWORD numBytesWritten = 0;

	if(State)
	{
		TRACE("Buzzer & LED Activated\n");
		m_pOwner->SystemInfo.SetWindowText("");
		m_pOwner->SystemInfo.SetWindowText("Buzzer & LED Activated.");
		m_pOwner->UpdateData(true);	
		WritePort(&m_activateBuzzerLed, 1, numBytesWritten);
	}
	else
	{
		TRACE("Buzzer & LED Deactivated\n");
		m_pOwner->SystemInfo.SetWindowText("");
		m_pOwner->SystemInfo.SetWindowText("Buzzer & LED Deactivated.");
		m_pOwner->UpdateData(true);	
		WritePort(&m_deactivateBuzzerLed, 1, numBytesWritten);
	}

	return true;
}

bool CPeripherals::Siren(unsigned int SirenNumber, bool State)
{
	DWORD numBytesWritten = 0;

	if(State)
	{
		switch(SirenNumber)
		{
			case 0: 
				TRACE("Siren 1 Activated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Siren 1 Activated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_activateSirens, 1, numBytesWritten);
			break;

			case 1: 
				TRACE("Siren 2 Activated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Siren 2 Activated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_activateSirens, 1, numBytesWritten);
			break;
		}
	}
	else
	{	
		switch(SirenNumber)
		{
			case 0: 
				TRACE("Siren 1 Deactivated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Siren 1 Deactivated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_deactivateSirens, 1, numBytesWritten);
			break;

			case 1: 
				TRACE("Siren 2 Deactivated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Siren 2 Deactivated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_deactivateSirens, 1, numBytesWritten);
			break;
		}
	}

	return true;
}

bool CPeripherals::FloodLight(unsigned int FloodLightNumber, bool State)
{
	DWORD numBytesWritten = 0;

	if(State)
	{
		switch(FloodLightNumber)
		{
			case 0:
				TRACE("Flood Light 1 Activated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Flood Light 1 Activated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_activateFloodlights, 1, numBytesWritten);
			break;

			case 1: 
				TRACE("Flood Light 2 Activated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Flood Light 2 Activated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_activateFloodlights, 1, numBytesWritten);
			break;
		}
	}
	else
	{	
		switch(FloodLightNumber)
		{
			case 0:
				TRACE("Flood Light 1 Deactivated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Flood Light 1 Deactivated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_deactivateFloodlights, 1, numBytesWritten);
			break;

			case 1: 
				TRACE("Flood Light 2 Deactivated\n");
				m_pOwner->SystemInfo.SetWindowText("");
				m_pOwner->SystemInfo.SetWindowText("Flood Light 2 Deactivated.");
				m_pOwner->UpdateData(true);
				WritePort(&m_deactivateFloodlights, 1, numBytesWritten);
			break;
		}
	}

	return true;
}

UINT RxPeripheralDataThread(LPVOID param)
{
	unsigned char streamBuffer	= 0;
	DWORD numBytesRead			= 0;
	DWORD numBytesWritten		= 0;

	CTouchScreenDlg* touchScreenDlg = (CTouchScreenDlg*) param;

	while(1)
	{
		if(touchScreenDlg->m_peripherals.m_killReadSerialPortThread)
		{
			AfxEndThread(0, true);
		}
		else
		{
			touchScreenDlg->m_peripherals.ReadPort(&streamBuffer, 1, numBytesRead);

			TRACE("Byte read from serial port: [%d]\n\n", streamBuffer);
			TRACE("Number of bytes read: %d\n", numBytesRead);

			touchScreenDlg->PostMessage(SENSOR_STATE_MSG, streamBuffer, 0);

			streamBuffer = 0;

			Sleep(25);
		}
	}

    return true;
}

UINT TestRxPeripheralDataThread(LPVOID param)
{
	CTouchScreenDlg* touchScreenDlg = (CTouchScreenDlg*) param;

	unsigned int tmp	= 0;
	unsigned int index	= 0;

	// Test variables.
	unsigned int motionSensor1Activated		= 0x0000000F;
	unsigned int motionSensor1Deactivated	= 0x00000000;
	unsigned int motionSensor2Activated		= 0x000000F0;
	unsigned int motionSensor2Deactivated	= 0x00000000;
	unsigned int doorSensor1Activated		= 0x00000F00;
	unsigned int doorSensor1Deactivated		= 0x00000000;
	unsigned int doorSensor2Activated		= 0x0000F000;
	unsigned int doorSensor2Deactivated		= 0x00000000;
	unsigned int windowSensorsActivated		= 0x000F0000;
	unsigned int windowSensorsDeactivated	= 0x00000000;

	//tmp = (motionSensor1Activated | doorSensor1Activated);

	// Ready to arm.
	tmp = 0;
	
	touchScreenDlg->PostMessage(SENSOR_STATE_MSG, tmp, 0);

	Sleep(2000);

	// Door sensor 1 triggered. Activate buzzer & led.
	tmp = doorSensor1Activated;

	touchScreenDlg->PostMessage(SENSOR_STATE_MSG, tmp, 0);

	Sleep(2000);

	// Motion Sensor 1 triggered. 
	tmp = motionSensor1Activated;

	while(index < 100)
	{
		// Post a message to the main window notifying it that motion sensor 1 was activated
		touchScreenDlg->PostMessage(SENSOR_STATE_MSG, tmp, 0);

		Sleep(1000);

		index++;
	}

	return true;
}