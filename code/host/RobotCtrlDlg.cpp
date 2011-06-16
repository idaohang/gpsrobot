// RobotCtrlDlg.cpp : implementation file
//

#include "stdafx.h"
#include "GPS Robot Host.h"
#include "RobotCtrlDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CRobotCtrlDlg dialog


CRobotCtrlDlg::CRobotCtrlDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRobotCtrlDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CRobotCtrlDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void CRobotCtrlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CRobotCtrlDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CRobotCtrlDlg, CDialog)
	//{{AFX_MSG_MAP(CRobotCtrlDlg)
		// NOTE: the ClassWizard will add message map macros here
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRobotCtrlDlg message handlers
