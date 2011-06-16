; CLW file contains information for the MFC ClassWizard

[General Info]
Version=1
LastClass=CControlDlg
LastTemplate=CDialog
NewFileInclude1=#include "stdafx.h"
NewFileInclude2=#include "GPS Robot Host.h"
LastPage=0

ClassCount=8
Class1=CGPSRobotHostApp
Class2=CGPSRobotHostDoc
Class3=CGPSRobotHostView
Class4=CMainFrame

ResourceCount=5
Resource1=IDR_MAINFRAME
Resource2=IDD_SERIAL_CONFIG
Class5=CAboutDlg
Resource3=IDD_ABOUTBOX
Resource4=IDD_PATH_CONFIG_DLG
Class6=CSerialPortDlg
Class7=CPathConfigDlg
Class8=CControlDlg
Resource5=IDD_CONTROL_DLG

[CLS:CGPSRobotHostApp]
Type=0
HeaderFile=GPS Robot Host.h
ImplementationFile=GPS Robot Host.cpp
Filter=N

[CLS:CGPSRobotHostDoc]
Type=0
HeaderFile=GPS Robot HostDoc.h
ImplementationFile=GPS Robot HostDoc.cpp
Filter=N

[CLS:CGPSRobotHostView]
Type=0
HeaderFile=GPS Robot HostView.h
ImplementationFile=GPS Robot HostView.cpp
Filter=C


[CLS:CMainFrame]
Type=0
HeaderFile=MainFrm.h
ImplementationFile=MainFrm.cpp
Filter=T
BaseClass=CFrameWnd
VirtualFilter=fWC
LastObject=ID_CONTROL_ROBOT




[CLS:CAboutDlg]
Type=0
HeaderFile=GPS Robot Host.cpp
ImplementationFile=GPS Robot Host.cpp
Filter=D
LastObject=CAboutDlg

[DLG:IDD_ABOUTBOX]
Type=1
Class=CAboutDlg
ControlCount=4
Control1=IDC_STATIC,static,1342177283
Control2=IDC_STATIC,static,1342308480
Control3=IDC_STATIC,static,1342308352
Control4=IDOK,button,1342373889

[MNU:IDR_MAINFRAME]
Type=1
Class=CMainFrame
Command1=ID_APP_EXIT
Command2=ID_SERIAL_CONFIG
Command3=ID_PATH_CONFIG
Command4=ID_CONTROL_ROBOT
Command5=ID_APP_ABOUT
CommandCount=5

[ACL:IDR_MAINFRAME]
Type=1
Class=CMainFrame
Command1=ID_FILE_NEW
Command2=ID_FILE_OPEN
Command3=ID_FILE_SAVE
Command4=ID_EDIT_UNDO
Command5=ID_EDIT_CUT
Command6=ID_EDIT_COPY
Command7=ID_EDIT_PASTE
Command8=ID_EDIT_UNDO
Command9=ID_EDIT_CUT
Command10=ID_EDIT_COPY
Command11=ID_EDIT_PASTE
Command12=ID_NEXT_PANE
Command13=ID_PREV_PANE
CommandCount=13

[DLG:IDD_PATH_CONFIG_DLG]
Type=1
Class=CPathConfigDlg
ControlCount=17
Control1=IDOK,button,1342275585
Control2=IDCANCEL,button,1342275584
Control3=IDC_STATIC,static,1342308352
Control4=IDC_DEGREE1,edit,1350639744
Control5=IDC_STATIC,static,1342308352
Control6=IDC_STATIC,static,1342308352
Control7=IDC_STATIC,static,1342308352
Control8=IDC_MINUTE1,edit,1350639744
Control9=IDC_STATIC,button,1342177287
Control10=IDC_STATIC,button,1342177287
Control11=IDC_DEGREE2,edit,1350639744
Control12=IDC_STATIC,static,1342308352
Control13=IDC_STATIC,static,1342308352
Control14=IDC_MINUTE2,edit,1350639744
Control15=IDC_STATIC,button,1342177287
Control16=IDC_STATIC,static,1342308352
Control17=IDC_STATIC,static,1342308352

[DLG:IDD_SERIAL_CONFIG]
Type=1
Class=CSerialPortDlg
ControlCount=6
Control1=IDOK,button,1342275585
Control2=IDCANCEL,button,1342275584
Control3=IDC_STATIC,static,1342308352
Control4=IDC_STATIC,static,1342308352
Control5=IDC_COM_PORT,combobox,1344339970
Control6=IDC_BAUD_RATE,combobox,1344339970

[DLG:IDD_CONTROL_DLG]
Type=1
Class=CControlDlg
ControlCount=11
Control1=IDOK,button,1342275585
Control2=IDC_STATIC,button,1342177287
Control3=IDC_STATIC,button,1342177287
Control4=IDC_STATIC,button,1342177287
Control5=IDC_STATIC,button,1342177287
Control6=IDC_DOWNLOAD_PATH,button,1342242816
Control7=IDC_START,button,1342242816
Control8=IDC_STOP,button,1342242816
Control9=IDC_ACTIVATE_LED,button,1342242816
Control10=IDC_DEACTIVATE_LEDS,button,1342242816
Control11=IDC_BLOW_UP,button,1342242816

[CLS:CSerialPortDlg]
Type=0
HeaderFile=SerialPortDlg.h
ImplementationFile=SerialPortDlg.cpp
BaseClass=CDialog
Filter=D
LastObject=CSerialPortDlg
VirtualFilter=dWC

[CLS:CPathConfigDlg]
Type=0
HeaderFile=PathConfigDlg.h
ImplementationFile=PathConfigDlg.cpp
BaseClass=CDialog
Filter=D
LastObject=CPathConfigDlg
VirtualFilter=dWC

[CLS:CControlDlg]
Type=0
HeaderFile=ControlDlg.h
ImplementationFile=ControlDlg.cpp
BaseClass=CDialog
Filter=D
LastObject=IDC_ACTIVATE_LED
VirtualFilter=dWC

