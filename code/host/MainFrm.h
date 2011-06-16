// MainFrm.h : interface of the CMainFrame class
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_MAINFRM_H__4261605B_7079_4591_97C5_27459B0F62F0__INCLUDED_)
#define AFX_MAINFRM_H__4261605B_7079_4591_97C5_27459B0F62F0__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "PathConfigDlg.h"
#include "ControlDlg.h"
#include "SerialPortDlg.h"

class CMainFrame : public CFrameWnd
{
	
protected: // create from serialization only
	CMainFrame();
	DECLARE_DYNCREATE(CMainFrame)

// Attributes
public:

	CSerialPortDlg	m_serialPortDlg;
	CPathConfigDlg	m_pathConfigDlg;
	CControlDlg			m_controlDlg;

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CMainFrame)
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CMainFrame();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

// Generated message map functions
protected:
	//{{AFX_MSG(CMainFrame)
	afx_msg void OnSerialConfig();
	afx_msg void OnPathConfig();
	afx_msg void OnControlRobot();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_MAINFRM_H__4261605B_7079_4591_97C5_27459B0F62F0__INCLUDED_)
