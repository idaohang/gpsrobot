// GPS Robot HostView.cpp : implementation of the CGPSRobotHostView class
//

#include "stdafx.h"
#include "GPS Robot Host.h"

#include "GPS Robot HostDoc.h"
#include "GPS Robot HostView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostView

IMPLEMENT_DYNCREATE(CGPSRobotHostView, CView)

BEGIN_MESSAGE_MAP(CGPSRobotHostView, CView)
	//{{AFX_MSG_MAP(CGPSRobotHostView)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostView construction/destruction

CGPSRobotHostView::CGPSRobotHostView()
{
	// TODO: add construction code here

}

CGPSRobotHostView::~CGPSRobotHostView()
{
}

BOOL CGPSRobotHostView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostView drawing

void CGPSRobotHostView::OnDraw(CDC* pDC)
{
	CGPSRobotHostDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	// TODO: add draw code for native data here
}

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostView diagnostics

#ifdef _DEBUG
void CGPSRobotHostView::AssertValid() const
{
	CView::AssertValid();
}

void CGPSRobotHostView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CGPSRobotHostDoc* CGPSRobotHostView::GetDocument() // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CGPSRobotHostDoc)));
	return (CGPSRobotHostDoc*)m_pDocument;
}
#endif //_DEBUG

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostView message handlers
