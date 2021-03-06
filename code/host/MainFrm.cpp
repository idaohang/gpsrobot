// MainFrm.cpp : implementation of the CMainFrame class
//

#include "stdafx.h"
#include "GPS Robot Host.h"

#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CMainFrame

IMPLEMENT_DYNCREATE(CMainFrame, CFrameWnd)

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWnd)
	//{{AFX_MSG_MAP(CMainFrame)
	ON_COMMAND(ID_SERIAL_CONFIG, OnSerialConfig)
	ON_COMMAND(ID_PATH_CONFIG, OnPathConfig)
	ON_COMMAND(ID_CONTROL_ROBOT, OnControlRobot)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CMainFrame construction/destruction

CMainFrame::CMainFrame()
{

}

CMainFrame::~CMainFrame()
{

}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWnd::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	cs.cx = 1400;
	cs.cy = 50;

	return TRUE;
}

/////////////////////////////////////////////////////////////////////////////
// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWnd::Dump(dc);
}

#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CMainFrame message handlers


void CMainFrame::OnSerialConfig() 
{
	if(m_serialPortDlg.DoModal() == IDOK)
	{

	}
}

void CMainFrame::OnPathConfig() 
{
	if(m_pathConfigDlg.DoModal() == IDOK)
	{

	}
}

void CMainFrame::OnControlRobot() 
{
	m_controlDlg.InitParentPtr(this);

	if(m_controlDlg.DoModal() == IDOK)
	{
	
	}
}
