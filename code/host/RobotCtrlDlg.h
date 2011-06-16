#if !defined(AFX_ROBOTCTRLDLG_H__A91B655B_0D2D_4CD2_A78C_55867C720625__INCLUDED_)
#define AFX_ROBOTCTRLDLG_H__A91B655B_0D2D_4CD2_A78C_55867C720625__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// RobotCtrlDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CRobotCtrlDlg dialog

class CRobotCtrlDlg : public CDialog
{
// Construction
public:
	CRobotCtrlDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CRobotCtrlDlg)
	enum { IDD = IDD_CONTROL_DLG };
		// NOTE: the ClassWizard will add data members here
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRobotCtrlDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CRobotCtrlDlg)
		// NOTE: the ClassWizard will add member functions here
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_ROBOTCTRLDLG_H__A91B655B_0D2D_4CD2_A78C_55867C720625__INCLUDED_)
