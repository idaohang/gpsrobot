// GPS Robot HostDoc.h : interface of the CGPSRobotHostDoc class
//
/////////////////////////////////////////////////////////////////////////////

#if !defined(AFX_GPSROBOTHOSTDOC_H__95806FDB_A337_44AE_9A1B_A4A68A0290EF__INCLUDED_)
#define AFX_GPSROBOTHOSTDOC_H__95806FDB_A337_44AE_9A1B_A4A68A0290EF__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


class CGPSRobotHostDoc : public CDocument
{
protected: // create from serialization only
	CGPSRobotHostDoc();
	DECLARE_DYNCREATE(CGPSRobotHostDoc)

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CGPSRobotHostDoc)
	public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~CGPSRobotHostDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	//{{AFX_MSG(CGPSRobotHostDoc)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_GPSROBOTHOSTDOC_H__95806FDB_A337_44AE_9A1B_A4A68A0290EF__INCLUDED_)
