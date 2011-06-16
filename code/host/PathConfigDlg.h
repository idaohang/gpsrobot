#if !defined(AFX_PATHCONFIGDLG_H__AAB0E354_C0B3_4773_9BD7_D5B0C28DFD07__INCLUDED_)
#define AFX_PATHCONFIGDLG_H__AAB0E354_C0B3_4773_9BD7_D5B0C28DFD07__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// PathConfigDlg.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CPathConfigDlg dialog

class CPathConfigDlg : public CDialog
{
// Construction
public:
	CPathConfigDlg(CWnd* pParent = NULL);   // standard constructor

// Dialog Data
	//{{AFX_DATA(CPathConfigDlg)
	enum { IDD = IDD_PATH_CONFIG_DLG };
	UINT	m_degree1;
	float	m_minute1;
	float	m_minute2;
	UINT	m_degree2;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CPathConfigDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CPathConfigDlg)
	virtual void OnOK();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_PATHCONFIGDLG_H__AAB0E354_C0B3_4773_9BD7_D5B0C28DFD07__INCLUDED_)
