// GPS Robot HostDoc.cpp : implementation of the CGPSRobotHostDoc class
//

#include "stdafx.h"
#include "GPS Robot Host.h"

#include "GPS Robot HostDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostDoc

IMPLEMENT_DYNCREATE(CGPSRobotHostDoc, CDocument)

BEGIN_MESSAGE_MAP(CGPSRobotHostDoc, CDocument)
	//{{AFX_MSG_MAP(CGPSRobotHostDoc)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostDoc construction/destruction

CGPSRobotHostDoc::CGPSRobotHostDoc()
{
	// TODO: add one-time construction code here

}

CGPSRobotHostDoc::~CGPSRobotHostDoc()
{
}

BOOL CGPSRobotHostDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}



/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostDoc serialization

void CGPSRobotHostDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostDoc diagnostics

#ifdef _DEBUG
void CGPSRobotHostDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CGPSRobotHostDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostDoc commands
