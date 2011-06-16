//{{AFX_INCLUDES()
//#include "NiGraph3D.h"
//#include "NiGraph.h"
//#include "NiKnob.h"
//}}AFX_INCLUDES
#if !defined(AFX_CONTROLDLG_H__674C8C36_5C46_4886_9047_CD9D8EDD2CB3__INCLUDED_)
#define AFX_CONTROLDLG_H__674C8C36_5C46_4886_9047_CD9D8EDD2CB3__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// ControlDlg.h : header file
//

//
// Structures
//

enum PACKET_TYPE
{
	DOWNLOAD_PATH = 1,
	START_NAVIGATION,
	STOP_NAVIGATION,
	ACTIVATE_LEDS,
	DEACTIVATE_LEDS
};

struct Packet
{
	unsigned char uartSyncByte;
	unsigned char startByte;
	unsigned char	packetType;
	unsigned char packetDataSize;
	unsigned char data[16];
	unsigned char checksum;
	unsigned char packetEnd;
};

//
// Constants
//

#define SERIALPORT					"COM4"
#define BAUDRATE						9600
#define SERIAL_BUFFER_SIZE	1024

#define UART_SYNC_BYTE			255
#define START_BYTE					254

#define DOWNLOAD_PATH				1
#define START_NAVIGATION		2
#define STOP_NAVIGATION			3
#define ACTIVATE_LEDS				4
#define DEACTIVATE_LEDS			5

#define PACKET_END					0x0a 

// Friendly Class
class CMainFrame;

/////////////////////////////////////////////////////////////////////////////
// CControlDlg dialog

class CControlDlg : public CDialog
{
// Construction
public:
	void InitParentPtr(CMainFrame* pOwner);
	void InitControlDlg();
	bool ClosePort();
	void WritePort(unsigned char* Data, int Length, DWORD NumBytesWritten);
	void ReadPort(unsigned char* Data, int Length, DWORD NumBytesRead);
	bool InitSerialPort();
	
	CControlDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CControlDlg)
	enum { IDD = IDD_CONTROL_DLG };
	CButton	m_downloadPath;
	CButton	m_deactivateLEDs;
	CButton	m_blowUp;
	CButton	m_activateLEDs;
//	CNiGraph3D	m_test;
//	CNiGraph	m_robotPath;
//	CNiKnob	m_batteryVoltage;
//	CNiKnob	m_robotSpeed;
	//}}AFX_DATA

	CMainFrame* m_pOwner;

	// Summons death of serial thread.
	bool m_killReadSerialPortThread;

private:

	// Serial port handle.
	HANDLE m_serialPortHandle;

  // Flag used to represent state of serial port.
	bool m_serialPortOpen;

	// Data packet
	struct Packet m_txPacket;
	struct Packet m_rxPacket;

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CControlDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CControlDlg)
	afx_msg void OnDownloadPath();
	virtual void OnOK();
	afx_msg void OnStart();
	afx_msg void OnStop();
	afx_msg void OnDeactivateLeds();
	afx_msg void OnBlowUp();
	afx_msg void OnActivateLed();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_CONTROLDLG_H__674C8C36_5C46_4886_9047_CD9D8EDD2CB3__INCLUDED_)
