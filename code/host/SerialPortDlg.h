#if !defined(AFX_SERIALPORTDLG_H__37B5CFEE_2AD3_4CAA_A3F0_DE7039F343D3__INCLUDED_)
#define AFX_SERIALPORTDLG_H__37B5CFEE_2AD3_4CAA_A3F0_DE7039F343D3__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// SerialPortDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CSerialPortDlg dialog

class CSerialPortDlg : public CDialog
{
// Construction
public:
	CSerialPortDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CSerialPortDlg)
	enum { IDD = IDD_SERIAL_CONFIG };
	CString	baudRate;
	CString	comPort;
	//}}AFX_DATA

	int			m_baudRate;
	CString	m_comPort;

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CSerialPortDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CSerialPortDlg)
	virtual void OnOK();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_SERIALPORTDLG_H__37B5CFEE_2AD3_4CAA_A3F0_DE7039F343D3__INCLUDED_)
