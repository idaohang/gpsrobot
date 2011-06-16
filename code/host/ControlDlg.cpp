// ControlDlg.cpp : implementation file
//

#include "stdafx.h"
#include "GPS Robot Host.h"
#include "ControlDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// Global Threads
UINT RxPeripheralDataThread(LPVOID param);

/////////////////////////////////////////////////////////////////////////////
// CControlDlg dialog


CControlDlg::CControlDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CControlDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CControlDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT

	m_serialPortHandle			    = 0;
	m_serialPortOpen			      = false;
	m_killReadSerialPortThread	= false;

	InitControlDlg();
}


void CControlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CControlDlg)
	DDX_Control(pDX, IDC_DOWNLOAD_PATH, m_downloadPath);
	DDX_Control(pDX, IDC_DEACTIVATE_LEDS, m_deactivateLEDs);
	DDX_Control(pDX, IDC_BLOW_UP, m_blowUp);
	DDX_Control(pDX, IDC_ACTIVATE_LED, m_activateLEDs);
//	DDX_Control(pDX, IDC_ROBOT_PATH, m_robotPath);
//	DDX_Control(pDX, IDC_BATTERY_VOLTAGE, m_batteryVoltage);
//	DDX_Control(pDX, IDC_ROBOT_SPEED, m_robotSpeed);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CControlDlg, CDialog)
	//{{AFX_MSG_MAP(CControlDlg)
	ON_BN_CLICKED(IDC_DOWNLOAD_PATH, OnDownloadPath)
	ON_BN_CLICKED(IDC_START, OnStart)
	ON_BN_CLICKED(IDC_STOP, OnStop)
	ON_BN_CLICKED(IDC_DEACTIVATE_LEDS, OnDeactivateLeds)
	ON_BN_CLICKED(IDC_BLOW_UP, OnBlowUp)
	ON_BN_CLICKED(IDC_ACTIVATE_LED, OnActivateLed)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CControlDlg message handlers

void CControlDlg::InitParentPtr(CMainFrame *pOwner)
{
	m_pOwner = pOwner;
}

void CControlDlg::InitControlDlg()
{
	m_killReadSerialPortThread = false;

	InitSerialPort();

	TRACE("Rx Serial Communications Thread Started.\n");
}

bool CControlDlg::InitSerialPort()
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
		PortDCB.BaudRate          = BAUDRATE;						// Current baud 
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

	m_killReadSerialPortThread = false;

	// Create a read thread for reading data from the communication port.
	AfxBeginThread(RxPeripheralDataThread, this, THREAD_PRIORITY_TIME_CRITICAL);

	m_serialPortOpen = true;

	return TRUE;
}

void CControlDlg::ReadPort(unsigned char *Data, int Length, DWORD NumBytesRead)
{
	DWORD NumBytes = 0;

	if(!ReadFile(m_serialPortHandle, Data, Length, &NumBytes, NULL))								
	{
		TRACE("\nreadPort Failed!");
	}
	
	TRACE("\nRead %d bytes from the serial port.\n", NumBytes);
}

void CControlDlg::WritePort(unsigned char *Data, int Length, DWORD NumBytesWritten)
{
	if(!WriteFile(m_serialPortHandle, Data, Length, &NumBytesWritten, NULL))								
	{
		TRACE("\nwritePort Failed!");
	}
	
	TRACE("\nWrote %d bytes to the serial port", NumBytesWritten);
}

bool CControlDlg::ClosePort()
{
	WSACleanup();

	m_killReadSerialPortThread = true;

	// Give thread time to finish and exit.
	Sleep(500);

	if(m_serialPortOpen)
	{
		CloseHandle(m_serialPortHandle);
	}

	m_serialPortOpen = false;

	return true;
}

UINT RxPeripheralDataThread(LPVOID param)
{
	unsigned char streamBuffer	= 0;
	DWORD numBytesRead					= 0;
	DWORD numBytesWritten				= 0;

	CControlDlg* controlDlg = (CControlDlg*) param;

	while(1)
	{
		if(controlDlg->m_killReadSerialPortThread)
		{
			AfxEndThread(0, true);
		}
		else
		{
			controlDlg->ReadPort(&streamBuffer, 1, numBytesRead);

			TRACE("Byte read from serial port: [%d]\n\n", streamBuffer);
			TRACE("Number of bytes read: %d\n", numBytesRead);

			//controlDlg->PostMessage(SENSOR_STATE_MSG, streamBuffer, 0);

			streamBuffer = 0;

			Sleep(25);
		}
	}

	return true;
}

void CControlDlg::OnOK() 
{
	ClosePort();
	
	CDialog::OnOK();
}

void CControlDlg::OnDownloadPath() 
{
	DWORD numBytesWritten	= 0;

	// Construct packet
	m_txPacket.uartSyncByte		= UART_SYNC_BYTE;
	m_txPacket.startByte			= START_BYTE;
	m_txPacket.packetType			= DOWNLOAD_PATH;
	m_txPacket.packetDataSize	= 16;
	memset(m_txPacket.data, 0, sizeof(m_txPacket.data));
	m_txPacket.checksum				= 0;
	m_txPacket.packetEnd			= PACKET_END;

	// Transmit packet
	WritePort((unsigned char*) &m_txPacket, sizeof(m_txPacket), numBytesWritten);
}

void CControlDlg::OnStart() 
{
	DWORD numBytesWritten	= 0;

	// Construct packet
	m_txPacket.uartSyncByte		= UART_SYNC_BYTE;
	m_txPacket.startByte			= START_BYTE;
	m_txPacket.packetType			= START_NAVIGATION;
	m_txPacket.packetDataSize	= 16;
	memset(m_txPacket.data, 0, sizeof(m_txPacket.data));
	m_txPacket.checksum				= 0;
	m_txPacket.packetEnd			= PACKET_END;

	// Transmit packet
	WritePort((unsigned char*) &m_txPacket, sizeof(m_txPacket), numBytesWritten);
}

void CControlDlg::OnStop() 
{
	DWORD numBytesWritten	= 0;

	// Construct packet
	m_txPacket.uartSyncByte		= UART_SYNC_BYTE;
	m_txPacket.startByte			= START_BYTE;
	m_txPacket.packetType			= STOP_NAVIGATION;
	m_txPacket.packetDataSize	= 16;
	memset(m_txPacket.data, 0, sizeof(m_txPacket.data));
	m_txPacket.checksum				= 0;
	m_txPacket.packetEnd			= PACKET_END;

	// Transmit packet
	WritePort((unsigned char*) &m_txPacket, sizeof(m_txPacket), numBytesWritten);
}

void CControlDlg::OnActivateLed() 
{
	DWORD numBytesWritten	= 0;

	// Construct packet
	m_txPacket.uartSyncByte		= UART_SYNC_BYTE;
	m_txPacket.startByte			= START_BYTE;
	m_txPacket.packetType			= ACTIVATE_LEDS;
	m_txPacket.packetDataSize	= 16;
	memset(m_txPacket.data, 0, sizeof(m_txPacket.data));
	m_txPacket.checksum				= 0;
	m_txPacket.packetEnd			= PACKET_END;

	// Transmit packet
	WritePort((unsigned char*) &m_txPacket, sizeof(m_txPacket), numBytesWritten);
}

void CControlDlg::OnDeactivateLeds() 
{
	DWORD numBytesWritten	= 0;

	// Construct packet
	m_txPacket.uartSyncByte		= UART_SYNC_BYTE;
	m_txPacket.startByte			= START_BYTE;
	m_txPacket.packetType			= DEACTIVATE_LEDS;
	m_txPacket.packetDataSize	= 16;
	memset(m_txPacket.data, 0, sizeof(m_txPacket.data));
	m_txPacket.checksum				= 0;
	m_txPacket.packetEnd			= PACKET_END;

	// Transmit packet
	WritePort((unsigned char*) &m_txPacket, sizeof(m_txPacket), numBytesWritten);	
}

void CControlDlg::OnBlowUp() 
{
	// Charts data points
//	m_robotPath.ChartXY(1,1);
//	m_robotPath.ChartXY(1,2);	
//	m_robotPath.ChartXY(1,3);	
	UpdateData(true);
}

