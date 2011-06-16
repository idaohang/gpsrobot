// SerialPortDlg.cpp : implementation file
//

#include "stdafx.h"
#include "GPS Robot Host.h"
#include "SerialPortDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CSerialPortDlg dialog


CSerialPortDlg::CSerialPortDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CSerialPortDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CSerialPortDlg)
	baudRate = _T("9600");
	comPort = _T("COM4");
	//}}AFX_DATA_INIT

	m_baudRate = 9600;
	
	m_comPort.Empty();
	m_comPort.Insert(0, "COM4");
}


void CSerialPortDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CSerialPortDlg)
	DDX_CBString(pDX, IDC_BAUD_RATE, baudRate);
	DDX_CBString(pDX, IDC_COM_PORT, comPort);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CSerialPortDlg, CDialog)
	//{{AFX_MSG_MAP(CSerialPortDlg)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CSerialPortDlg message handlers

void CSerialPortDlg::OnOK() 
{
	if(baudRate.Compare("4800") == 0)
	{
		m_baudRate = 4800;
	}
	else if(baudRate.Compare("9600") == 0)
	{
		m_baudRate = 9600;
	}
	else if(baudRate.Compare("115200") == 0)
	{
		m_baudRate = 115200;
	}

	m_comPort	= comPort;
	
	CDialog::OnOK();
}
