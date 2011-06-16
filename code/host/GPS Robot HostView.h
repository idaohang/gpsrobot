// GPS Robot HostView.h : interface of the CGPSRobotHostView class
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_GPSROBOTHOSTVIEW_H__DD05750F_CF95_4B43_8807_E78225665E02__INCLUDED_)
#define AFX_GPSROBOTHOSTVIEW_H__DD05750F_CF95_4B43_8807_E78225665E02__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


class CGPSRobotHostView : public CView
{
protected: // create from serialization only
	CGPSRobotHostView();
	DECLARE_DYNCREATE(CGPSRobotHostView)

// Attributes
public:
	CGPSRobotHostDoc* GetDocument();

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CGPSRobotHostView)
	public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
	protected:
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CGPSRobotHostView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	//{{AFX_MSG(CGPSRobotHostView)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in GPS Robot HostView.cpp
inline CGPSRobotHostDoc* CGPSRobotHostView::GetDocument()
   { return (CGPSRobotHostDoc*)m_pDocument; }
#endif

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_GPSROBOTHOSTVIEW_H__DD05750F_CF95_4B43_8807_E78225665E02__INCLUDED_)
