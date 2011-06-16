// PathConfigDlg.cpp : implementation file
//

#include "stdafx.h"
#include "GPS Robot Host.h"
#include "PathConfigDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CPathConfigDlg dialog


CPathConfigDlg::CPathConfigDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CPathConfigDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CPathConfigDlg)
	m_degree1 = 0;
	m_minute1 = 0.0f;
	m_minute2 = 0.0f;
	m_degree2 = 0;
	//}}AFX_DATA_INIT
}


void CPathConfigDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CPathConfigDlg)
	DDX_Text(pDX, IDC_DEGREE1, m_degree1);
	DDX_Text(pDX, IDC_MINUTE1, m_minute1);
	DDX_Text(pDX, IDC_MINUTE2, m_minute2);
	DDX_Text(pDX, IDC_DEGREE2, m_degree2);
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(CPathConfigDlg, CDialog)
	//{{AFX_MSG_MAP(CPathConfigDlg)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CPathConfigDlg message handlers

void CPathConfigDlg::OnOK() 
{
	
	
	CDialog::OnOK();
}
