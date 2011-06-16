// GPS Robot Host.h : main header file for the GPS ROBOT HOST application
//

#if !defined(AFX_GPSROBOTHOST_H__7E338AEF_D086_4352_9D85_5541DD22157E__INCLUDED_)
#define AFX_GPSROBOTHOST_H__7E338AEF_D086_4352_9D85_5541DD22157E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"       // main symbols

/////////////////////////////////////////////////////////////////////////////
// CGPSRobotHostApp:
// See GPS Robot Host.cpp for the implementation of this class
//

class CGPSRobotHostApp : public CWinApp
{
public:
	CGPSRobotHostApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CGPSRobotHostApp)
	public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation
	//{{AFX_MSG(CGPSRobotHostApp)
	afx_msg void OnAppAbout();
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_GPSROBOTHOST_H__7E338AEF_D086_4352_9D85_5541DD22157E__INCLUDED_)
