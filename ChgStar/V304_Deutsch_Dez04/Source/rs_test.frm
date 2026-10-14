VERSION 5.00
Object = "{648A5603-2C6E-101B-82B6-000000000014}#1.1#0"; "MSCOMM32.OCX"
Object = "{86CF1D34-0C5F-11D2-A9FC-0000F8754DA1}#2.0#0"; "MSCOMCT2.OCX"
Begin VB.Form rs_test 
   Caption         =   "Verbindungs Tester"
   ClientHeight    =   9555
   ClientLeft      =   60
   ClientTop       =   345
   ClientWidth     =   8055
   LinkTopic       =   "Form1"
   ScaleHeight     =   9555
   ScaleWidth      =   8055
   StartUpPosition =   3  'Windows-Standard
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   2
      Left            =   6600
      TabIndex        =   98
      Top             =   1680
      Width           =   1095
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   9
      Left            =   7200
      TabIndex        =   90
      Text            =   "3"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   8
      Left            =   6480
      TabIndex        =   88
      Text            =   "220"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   7
      Left            =   6000
      TabIndex        =   86
      Text            =   "0"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   6
      Left            =   5280
      TabIndex        =   85
      Text            =   "71"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   5
      Left            =   4800
      TabIndex        =   83
      Text            =   "70"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      DataField       =   "0"
      Height          =   285
      Index           =   4
      Left            =   4080
      TabIndex        =   82
      Text            =   "10"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   3
      Left            =   3360
      TabIndex        =   80
      Text            =   "1"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   2
      Left            =   2880
      TabIndex        =   79
      Text            =   "0"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   1
      Left            =   2160
      TabIndex        =   77
      Text            =   "66"
      Top             =   3960
      Width           =   375
   End
   Begin VB.CommandButton FolgenderBfSend 
      Caption         =   "&Folgenden Befehl senden"
      Height          =   615
      Left            =   120
      TabIndex        =   76
      Top             =   3840
      Width           =   975
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   0
      Left            =   1440
      TabIndex        =   74
      Text            =   "2"
      Top             =   3960
      Width           =   375
   End
   Begin VB.TextBox TestBoxB 
      Height          =   285
      Left            =   2040
      TabIndex        =   71
      Text            =   "44"
      Top             =   240
      Width           =   615
   End
   Begin VB.TextBox TestBoxA 
      Height          =   285
      Left            =   1320
      TabIndex        =   70
      Text            =   "55"
      Top             =   240
      Width           =   615
   End
   Begin VB.CommandButton TestBefehl 
      Caption         =   "Test_Befehl"
      Height          =   495
      Left            =   1320
      TabIndex        =   69
      Top             =   600
      Width           =   1335
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   3
      Left            =   2400
      TabIndex        =   62
      Text            =   "0"
      Top             =   5640
      Width           =   885
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   2
      Left            =   960
      TabIndex        =   61
      Text            =   "0"
      Top             =   5640
      Width           =   885
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   1
      Left            =   2400
      TabIndex        =   59
      Text            =   "0"
      Top             =   8040
      Width           =   915
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   4
      Left            =   5640
      TabIndex        =   58
      Text            =   "24"
      Top             =   3000
      Width           =   615
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   4
      Left            =   4680
      TabIndex        =   57
      Text            =   "69"
      Top             =   3000
      Width           =   615
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   4
      Left            =   6600
      TabIndex        =   56
      Top             =   2880
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl &senden"
      Height          =   495
      Index           =   4
      Left            =   3720
      TabIndex        =   55
      Top             =   2880
      Width           =   735
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   4
      Left            =   6600
      TabIndex        =   54
      Text            =   "6"
      Top             =   3120
      Width           =   1095
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   4
      Left            =   2880
      TabIndex        =   53
      Text            =   "OK Set"
      Top             =   3000
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   3
      Left            =   2880
      TabIndex        =   52
      Text            =   "Error on Set"
      Top             =   2400
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   2
      Left            =   2880
      TabIndex        =   51
      Text            =   "Send Prop"
      Top             =   1800
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   1
      Left            =   2880
      TabIndex        =   50
      Text            =   "Get Prop"
      Top             =   1200
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   0
      Left            =   2880
      TabIndex        =   49
      Text            =   "Set Prop"
      Top             =   480
      Width           =   735
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   3
      Left            =   6600
      TabIndex        =   48
      Text            =   "Text6"
      Top             =   2520
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl senden"
      Height          =   495
      Index           =   3
      Left            =   3720
      TabIndex        =   47
      Top             =   2280
      Width           =   735
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   3
      Left            =   6600
      TabIndex        =   46
      Text            =   "32pq"
      Top             =   2280
      Width           =   1095
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   3
      Left            =   4680
      TabIndex        =   45
      Text            =   "68"
      Top             =   2400
      Width           =   615
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   3
      Left            =   5640
      TabIndex        =   44
      Text            =   "23"
      Top             =   2400
      Width           =   615
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   2
      Left            =   6600
      TabIndex        =   43
      Top             =   1920
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "&Befehl senden"
      Height          =   495
      Index           =   2
      Left            =   3720
      TabIndex        =   42
      Top             =   1680
      Width           =   735
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   2
      Left            =   4680
      TabIndex        =   41
      Text            =   "67"
      Top             =   1800
      Width           =   615
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   2
      Left            =   5640
      TabIndex        =   40
      Text            =   "22"
      Top             =   1800
      Width           =   615
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   1
      Left            =   6600
      TabIndex        =   39
      Top             =   1320
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl senden"
      Height          =   495
      Index           =   1
      Left            =   3720
      TabIndex        =   38
      Top             =   1080
      Width           =   735
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   1
      Left            =   6600
      TabIndex        =   37
      Top             =   1080
      Width           =   1095
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   1
      Left            =   4680
      TabIndex        =   36
      Text            =   "66"
      Top             =   1200
      Width           =   615
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   1
      Left            =   5640
      TabIndex        =   35
      Text            =   "21"
      Top             =   1200
      Width           =   615
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   0
      Left            =   6600
      TabIndex        =   32
      Text            =   "22"
      Top             =   600
      Width           =   1095
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   0
      Left            =   960
      TabIndex        =   31
      Text            =   "0"
      Top             =   8040
      Width           =   915
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   285
      Index           =   0
      Left            =   720
      TabIndex        =   30
      Top             =   8040
      Width           =   270
      _ExtentX        =   476
      _ExtentY        =   503
      _Version        =   393216
      BuddyControl    =   "Text1(0)"
      BuddyDispid     =   196614
      BuddyIndex      =   0
      OrigLeft        =   120
      OrigTop         =   1200
      OrigRight       =   390
      OrigBottom      =   1485
      Max             =   1000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   65547
      Enabled         =   -1  'True
   End
   Begin VB.CommandButton ProfilGet 
      Caption         =   "Ganzes Profil abfragen"
      Height          =   495
      Left            =   1440
      TabIndex        =   29
      Top             =   2520
      Width           =   1095
   End
   Begin VB.CheckBox CheckNLG 
      Caption         =   "Ich bin das NLG"
      Height          =   495
      Left            =   120
      TabIndex        =   28
      Top             =   840
      Width           =   975
   End
   Begin VB.CommandButton SendProfile 
      Caption         =   "Ganzes Profil senden"
      Height          =   495
      Left            =   1440
      TabIndex        =   27
      Top             =   1920
      Width           =   1095
   End
   Begin VB.TextBox StrSendDaten 
      Height          =   375
      Left            =   3600
      TabIndex        =   25
      Top             =   5640
      Width           =   4095
   End
   Begin VB.TextBox ByteSendDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   21
      Top             =   6120
      Width           =   6975
   End
   Begin VB.TextBox HexSendDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   20
      Top             =   6600
      Width           =   6975
   End
   Begin VB.TextBox ByteHexDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   16
      Top             =   9000
      Width           =   6975
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   0
      Left            =   5640
      TabIndex        =   14
      Text            =   "20"
      Top             =   480
      Width           =   615
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   0
      Left            =   4680
      TabIndex        =   12
      Text            =   "65"
      Top             =   480
      Width           =   615
   End
   Begin VB.TextBox StrEmpfDaten 
      Height          =   375
      Left            =   3600
      TabIndex        =   11
      Top             =   8040
      Width           =   4095
   End
   Begin VB.TextBox ByteEmpfDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   10
      Top             =   8520
      Width           =   6975
   End
   Begin VB.CommandButton test1 
      Caption         =   "&Verbinden"
      Default         =   -1  'True
      Height          =   495
      Left            =   120
      TabIndex        =   6
      Top             =   1920
      Width           =   1095
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   0
      Left            =   6600
      TabIndex        =   5
      Top             =   360
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl senden"
      Height          =   735
      Index           =   0
      Left            =   3720
      TabIndex        =   4
      Top             =   120
      Width           =   735
   End
   Begin VB.CommandButton test3 
      Caption         =   "Verbindung Abbrechen"
      Height          =   495
      Left            =   120
      TabIndex        =   3
      Top             =   1320
      Width           =   1095
   End
   Begin VB.TextBox Text5 
      Height          =   285
      Left            =   120
      TabIndex        =   2
      Top             =   3360
      Width           =   1095
   End
   Begin VB.CommandButton Command4 
      Caption         =   "Verbindung &abfragen"
      Height          =   495
      Left            =   120
      TabIndex        =   1
      Top             =   2520
      Width           =   1095
   End
   Begin VB.CommandButton Command6 
      Caption         =   "Fix Befehl senden"
      Height          =   495
      Left            =   1440
      TabIndex        =   0
      Top             =   1320
      Width           =   1095
   End
   Begin MSCommLib.MSComm MSComm1 
      Left            =   1680
      Top             =   3120
      _ExtentX        =   1005
      _ExtentY        =   1005
      _Version        =   393216
      DTREnable       =   -1  'True
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   285
      Index           =   1
      Left            =   2160
      TabIndex        =   60
      Top             =   8040
      Width           =   270
      _ExtentX        =   476
      _ExtentY        =   503
      _Version        =   393216
      BuddyControl    =   "Text1(1)"
      BuddyDispid     =   196614
      BuddyIndex      =   1
      OrigLeft        =   120
      OrigTop         =   1560
      OrigRight       =   390
      OrigBottom      =   1845
      Max             =   10000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   65547
      Enabled         =   -1  'True
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   285
      Index           =   2
      Left            =   720
      TabIndex        =   63
      Top             =   5640
      Width           =   270
      _ExtentX        =   476
      _ExtentY        =   503
      _Version        =   393216
      BuddyControl    =   "Text1(2)"
      BuddyDispid     =   196614
      BuddyIndex      =   2
      OrigLeft        =   120
      OrigTop         =   1200
      OrigRight       =   390
      OrigBottom      =   1485
      Max             =   1000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   0
      Enabled         =   -1  'True
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   285
      Index           =   3
      Left            =   2160
      TabIndex        =   64
      Top             =   5640
      Width           =   270
      _ExtentX        =   476
      _ExtentY        =   503
      _Version        =   393216
      BuddyControl    =   "Text1(3)"
      BuddyDispid     =   196614
      BuddyIndex      =   3
      OrigLeft        =   120
      OrigTop         =   1560
      OrigRight       =   390
      OrigBottom      =   1845
      Max             =   10000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   0
      Enabled         =   -1  'True
   End
   Begin VB.Label Label18 
      Caption         =   "2"
      Height          =   255
      Index           =   12
      Left            =   1560
      TabIndex        =   97
      Top             =   3720
      Width           =   135
   End
   Begin VB.Label Label18 
      Caption         =   "10"
      Height          =   255
      Index           =   11
      Left            =   4200
      TabIndex        =   96
      Top             =   3720
      Width           =   375
   End
   Begin VB.Label Label18 
      Caption         =   "3"
      Height          =   255
      Index           =   10
      Left            =   7320
      TabIndex        =   95
      Top             =   3720
      Width           =   135
   End
   Begin VB.Label Label18 
      Caption         =   "G"
      Height          =   255
      Index           =   9
      Left            =   5400
      TabIndex        =   94
      Top             =   3720
      Width           =   135
   End
   Begin VB.Label Label18 
      Caption         =   "F"
      Height          =   255
      Index           =   8
      Left            =   4920
      TabIndex        =   93
      Top             =   3720
      Width           =   135
   End
   Begin VB.Label Label18 
      Caption         =   "Get"
      Height          =   255
      Index           =   7
      Left            =   2160
      TabIndex        =   92
      Top             =   3720
      Width           =   375
   End
   Begin VB.Label Label18 
      Caption         =   "Op_ID"
      Height          =   255
      Index           =   6
      Left            =   2040
      TabIndex        =   91
      Top             =   4320
      Width           =   495
   End
   Begin VB.Label Label18 
      Caption         =   "End"
      Height          =   255
      Index           =   5
      Left            =   7200
      TabIndex        =   89
      Top             =   4320
      Width           =   375
   End
   Begin VB.Label Label18 
      Caption         =   "Value 1 / 2"
      Height          =   255
      Index           =   4
      Left            =   4680
      TabIndex        =   87
      Top             =   4320
      Width           =   1095
   End
   Begin VB.Label Label18 
      Caption         =   "CRC HI / Lo"
      Height          =   255
      Index           =   3
      Left            =   6000
      TabIndex        =   84
      Top             =   4320
      Width           =   975
   End
   Begin VB.Label Label18 
      Caption         =   "Length"
      Height          =   255
      Index           =   2
      Left            =   3960
      TabIndex        =   81
      Top             =   4320
      Width           =   495
   End
   Begin VB.Label Label18 
      Caption         =   "Prop HI / Lo"
      Height          =   255
      Index           =   1
      Left            =   2760
      TabIndex        =   78
      Top             =   4320
      Width           =   1095
   End
   Begin VB.Label Label18 
      Caption         =   "Start"
      Height          =   255
      Index           =   0
      Left            =   1440
      TabIndex        =   75
      Top             =   4320
      Width           =   375
   End
   Begin VB.Label Label16 
      Caption         =   "Hex Hi"
      Height          =   255
      Index           =   1
      Left            =   1320
      TabIndex        =   73
      Top             =   0
      Width           =   615
   End
   Begin VB.Label Label16 
      Caption         =   "Hex Lo"
      Height          =   255
      Index           =   0
      Left            =   2040
      TabIndex        =   72
      Top             =   0
      Width           =   615
   End
   Begin VB.Label Label7 
      Caption         =   "Bytes"
      Height          =   375
      Index           =   3
      Left            =   2400
      TabIndex        =   68
      Top             =   5400
      Width           =   735
   End
   Begin VB.Label Label7 
      Caption         =   "Frames"
      Height          =   255
      Index           =   2
      Left            =   1080
      TabIndex        =   67
      Top             =   5400
      Width           =   735
   End
   Begin VB.Label Label7 
      Caption         =   "Bytes"
      Height          =   255
      Index           =   1
      Left            =   2520
      TabIndex        =   66
      Top             =   7800
      Width           =   735
   End
   Begin VB.Label Label7 
      Caption         =   "Frames"
      Height          =   255
      Index           =   0
      Left            =   960
      TabIndex        =   65
      Top             =   7800
      Width           =   735
   End
   Begin VB.Label Label15 
      Caption         =   "Dez - Wert"
      Height          =   255
      Index           =   0
      Left            =   6720
      TabIndex        =   34
      Top             =   840
      Width           =   975
   End
   Begin VB.Label Label3 
      Caption         =   "oder"
      Height          =   255
      Index           =   0
      Left            =   7680
      TabIndex        =   33
      Top             =   480
      Width           =   375
   End
   Begin VB.Label Label14 
      Caption         =   "String"
      Height          =   255
      Left            =   4080
      TabIndex        =   26
      Top             =   5400
      Width           =   495
   End
   Begin VB.Label Label13 
      BackColor       =   &H00FFFFFF&
      Caption         =   "  Gesendete Daten"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Left            =   720
      TabIndex        =   24
      Top             =   5040
      Width           =   1935
   End
   Begin VB.Label Label12 
      Caption         =   "Dez"
      Height          =   255
      Left            =   240
      TabIndex        =   23
      Top             =   6240
      Width           =   375
   End
   Begin VB.Label Label11 
      Caption         =   "Hex"
      Height          =   255
      Left            =   240
      TabIndex        =   22
      Top             =   6720
      Width           =   375
   End
   Begin VB.Label Label10 
      Caption         =   "Hex"
      Height          =   255
      Left            =   240
      TabIndex        =   19
      Top             =   9120
      Width           =   375
   End
   Begin VB.Label Label9 
      Caption         =   "Dez"
      Height          =   255
      Left            =   240
      TabIndex        =   18
      Top             =   8640
      Width           =   375
   End
   Begin VB.Label Label8 
      Caption         =   "String"
      Height          =   255
      Left            =   3840
      TabIndex        =   17
      Top             =   7800
      Width           =   495
   End
   Begin VB.Label Label2 
      Caption         =   "Prop_ID (dez)"
      Height          =   255
      Index           =   0
      Left            =   5520
      TabIndex        =   15
      Top             =   120
      Width           =   1095
   End
   Begin VB.Label Label1 
      Caption         =   "OP_ID (dez)"
      Height          =   255
      Index           =   0
      Left            =   4560
      TabIndex        =   13
      Top             =   120
      Width           =   975
   End
   Begin VB.Label LabelEmpfDaten 
      BackColor       =   &H00FFFFFF&
      Caption         =   "  Empfangene Daten"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Left            =   720
      TabIndex        =   9
      Top             =   7440
      Width           =   1935
   End
   Begin VB.Label Label5 
      Caption         =   "Datenstring"
      Height          =   255
      Index           =   0
      Left            =   6720
      TabIndex        =   8
      Top             =   120
      Width           =   855
   End
   Begin VB.Label Label6 
      Caption         =   "Verbindung OK?"
      Height          =   255
      Left            =   120
      TabIndex        =   7
      Top             =   3120
      Width           =   1215
   End
   Begin VB.Image Image1 
      Height          =   720
      Left            =   240
      Picture         =   "rs_test.frx":0000
      Top             =   120
      Width           =   720
   End
End
Attribute VB_Name = "rs_test"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / rs_test                                                                 '
'---------------------------------------------------------------------------------------------------------------'
' MODUL BESCHREIBUNG                  / Fenster und Funktionen für die Kommunikation über RS232, ev. auch über  '
'                                     / andere Kommunikationskanäle. Momentan ist nur RS232 implementiert.      '
'                                     / Ein Teil der Funktionen ist allgemein gehalten, andere dienen nur zur   '
'                                     / Entwicklung / Test                                                      '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' HISTORY   / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' ERSTELLT  / Okt. '01  / Quang H.    / Rohfassung (Prinzipversuche)                                            '
' GEÄNDERT  / Nov. '01  / Beat G.     / Universalfunktionen eingefügt, allg. Protokoll für RS232 entworfen      '
'             Dez. '01  / Beat G.     / Senden/Empfangen Kennlinie, Frage-Antwort Logik                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' MODUL - LISTE                       / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
' MSComm1_OnComm                      / "Interrupt" - Funktion, empfängt und puffert ankommende Zeichen         '
' WrapString                          / Datenstrom mit Länge, CRC usw. versandbereit machen                     '
' CalculateCRC                        / Checksumme des zu sendenden Datenstroms berechnen                       '
' SendFrame                           / Datenstrom (Frame) byteweise an der seriellen Schnittstelle absetzen    '
' SendReceiveProfile                  / Kennlinie versenden oder empfangen                                      '
' WaitForReply                        / Antwort von NLG auf versendetes Frame empfangen und auswerten           '
' FnGetByteFromFrame                  / ein bestimmtes Byte aus einem Frame holen                               '
' CheckFrameLengthAndCRC              / Länge und Checksumme eines Frames auf Korrektheit prüfen                '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'
'
' Achtung:  Mangels sinnvoller Alternativen wurde die Datenhaltung des Sende - Streams mit einem modulglobalen Array
'           yOutStream realisiert; dito CRC
'
'

Option Explicit

Dim strInBuffer As String               ' String variabler Länge, hier kommen die empfangenen Daten rein
Dim strRecFrameBuf As String            ' empfangenes, komplettes Frame zwischenspeichern zum Verarbeiten
Dim out7 As Boolean
' rrr debug Dim raus2 As Boolean
Dim fenster As Boolean
Dim envoyer As Boolean
Dim test_string As String
Dim ret As Integer                      ' nur für Debug

Dim bBinNLG As Boolean                  ' bin ich das NLG?

Dim yOutStream(COM_MAX_LEN) As Byte     ' modulglobaler Array für den Out - Stream
Dim yHiCRC As Byte, yLoCRC As Byte      ' modulglobale Variablen für CRC Hi und Lo Byte



Private Sub CheckNLG_Click()
  bBinNLG = CheckNLG.Value              ' wenn angeklickt, spielt dieses Objekt NLG, sonst PC
End Sub



' bestimmtes Kommando absetzen, das hier "hart codiert" wird
Private Sub Command6_Click()
    
  test_string = (Chr$(55) & Chr$(53))                           ' String "75"
  Call WrapString(OP_SET, 0, BATT_NENN_SPANNG, test_string)     ' Batteriespannung
  Call SendFrame                                                ' soeben erstelltes Frame senden
  test_string = ""

End Sub

' einen komplett selbstgestrickten Befehl versenden
Private Sub FolgenderBfSend_Click()
Dim iCount As Integer

iCount = 0                                                          ' Dummy

For iCount = 0 To (Val(Text2(4).Text) - 1)                      ' Stringlänge bestimmen
  yOutStream(iCount) = Val(Text2(iCount).Text)                  ' Datenstrom mit gewünschten Bytes füllen
Next iCount

yOutStream(COM_MAX_LEN) = iCount

Call SendFrame                                                  ' Frame versenden

End Sub

' Initialisierung
Private Sub Form_Load()
  strInBuffer = ""
  out7 = True
  fenster = True
  envoyer = False
End Sub

' Kennlinie abfragen
Private Sub ProfilGet_Click()
  Call SendReceiveProfile(False)
End Sub

' Kennlinie senden
Private Sub SendProfile_Click()
  Call SendReceiveProfile(True)
End Sub

' Verbindung herstellen
' MSComm1 ist das Telefönli - Objekt
Private Sub test1_Click()
    
    If MSComm1.PortOpen = False Then
      MSComm1.CommPort = 1
      MSComm1.Settings = "19200,n,8,1"
      MSComm1.InputLen = 1
      MSComm1.RThreshold = 1
      MSComm1.SThreshold = 1
      MSComm1.PortOpen = True
      out7 = True
    Else
      MsgBox ("Es besteht schon eine Verbindung")
    End If
    
End Sub


' Verbindung abbrechen
Private Sub test3_Click()
    
  If MSComm1.PortOpen = True Then
    MSComm1.PortOpen = False
  Else
    MsgBox ("Comm1 noch nicht offen")
  End If
    
End Sub

' Senden eines bestimmten Kommandos, aus den "Sende - Fenstern" Feld

Private Sub test2_Click(Index As Integer)

  Dim strLoc As String
  Dim lLocVal As Long                                                   ' lokaler Wert aus "Dez" - Fensterli
  
  Text4(Index).Text = Trim(Text4(Index).Text)                           ' allfällige Leerzeichen abschneiden
  lLocVal = Val(Text6(Index).Text)                                      ' der numerische Wert aus dem Textfeld
  
  If (lLocVal > MAX_BYTE) Then
    strLoc = Chr(FnHiByteOfLong(lLocVal))                                 ' Hi Byte des numerischen Werts, falls vorhanden
  End If
  
  strLoc = strLoc & Chr(FnLoByteOfLong(lLocVal))                        ' Lo Byte des numerischen Werts
  
  'lLocHiVal = Fix(lLocVal / SHIFT_8_LEFT)                               ' Hi Byte des numerischen Werts
  'strLoc = Chr(Val(lLocHiVal))                                          ' Hi Byte als String
  'strLoc = strLoc & Chr(Fix(Val(lLocVal - (SHIFT_8_LEFT * lLocHiVal))))          ' Lo Byte, als String
  
  If (Text4(Index).Text = "") Then                                      ' numerischen Wert als..
    Call WrapString(Val(OPText(Index).Text), 0, Val(PropText(Index).Text), strLoc) '..Datenstring einpacken
  Else                                                                  ' sonst immer den Text im String - Feld..
    Call WrapString(Val(OPText(Index).Text), 0, Val(PropText(Index).Text), Text4(Index).Text) '..als Datenstring einpacken
  End If
  
  Call SendFrame                                                        ' Frame verschicken

End Sub

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Empfängt ankommende Zeichen, schiebt diese in den Buffer und erhöht dessen Zähler um 1; wenn gültiges Frame.. '
'..erkannt, d.h. Länge und Overhead stimmen, wird die Auswertung aufgerufen                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Static Sub MSComm1_OnComm()

Dim strEmpfString     As String                       ' Empfangs - String, wird aus den einzelnen Zeichen gebastelt
Static bBufOverflow   As Boolean                      ' zeigt an, wenn ein Überlauf im Empfangspuffer vorgekommen ist

  'Debug.Print MSComm1.CommEvent                      ' rrr braucht's nicht "sende das Kommunikatons - Ereignis ans Direktfenster"
    
  If MSComm1.CommEvent = comEvReceive Then            ' etwas empfangen
    
    strEmpfString = ""
    strEmpfString = MSComm1.Input                     ' Eingangspuffer holen / löschen
    UpDown1(1).Value = UpDown1(1).Value + 1           ' Empfangene Bytes / Ereignisse zählen
    
    strInBuffer = strInBuffer & strEmpfString         ' empfangenen String an Puffer anhängen
    
    ' Wenn Länge des Strings kleiner gleich max. Länge, ist's ok; sonst: Fehler
    If Len(strInBuffer) <= COM_MAX_LEN Then
      If (Len(strInBuffer) > COM_POS_VAL_START) Then  ' ab dieser Länge könnte es ein korrektes Frame sein
        If Asc(Right(strInBuffer, 1)) = COM_ETX Then  ' auf "ETX" (Ascii 3) triggern, also Ende Frame
          ' Wenn Länge des Strings wie angekündigt, dann..
          If Asc(Mid(strInBuffer, COM_POS_LENGTH + 1, 1)) = Len(strInBuffer) Then
'           If CheckNLG.Value = True Then
            strRecFrameBuf = strInBuffer              ' empfangenes, komplettes Frame puffern (noch nicht auf Korrektheit geprüft)
            Call Display_String(strRecFrameBuf)       ' empfangenen String anzeigen
'           End If
            g_bMsgReceived = True                     ' es ist eine Message (korrektes Frame) angekommen, für "PC"
            bBufOverflow = False                      ' alles ok
            strInBuffer = ""                          ' Buffer löschen
          End If
        End If
      End If
    Else                                              ' String länger als max erlaubt
      If (bBufOverflow = False) Then                  ' erstes Byte, das überläuft
        If SWITCH_DEBUG = True Then
          MsgBox ("Überlauf beim Empfangspuffer")     ' Empfangs - String zu lang
        End If
        bBufOverflow = True                           ' Überlauf neu
      Else                                            ' beim nächsten "Ende Paket" oder Buffer VIEL zu lang: Buffer löschen
        If ((Asc(Right(strInBuffer, 1)) = COM_ETX) Or (Len(strInBuffer) >= (2 * COM_MAX_LEN))) Then
          strInBuffer = ""                            ' Puffer löschen, neuer Versuch
          bBufOverflow = False                        ' Fehler ade, hab' ja gelöscht
        End If
      End If
    End If
    
  End If
    
End Sub

' Verbindung abfragen
Private Sub Command4_Click()
    If MSComm1.PortOpen Then
        Text5.Text = MSComm1.DSRHolding
    Else
        MsgBox ("Es besteht noch keine Verbindung")
    End If
    
End Sub



' String anzeigen, im Wesentlich Debug - Funktion
Private Sub Display_String(ByVal strShow As String)
   
  Dim Length As Integer                         ' Frame - Länge
  Dim iCount As Integer                         ' Zählervariable
  ' Initialisierungen
 
  StrEmpfDaten.Text = ""                        ' Feld "Empfangene Daten als String" löschen
  StrEmpfDaten.Refresh                          '
   
  ByteEmpfDaten.Text = Empty                    ' Feld "Empfangene Daten als Bytes in Dez" löschen
  ByteEmpfDaten.Refresh
   
  ByteHexDaten.Text = Empty                     ' Feld "Empfangene Daten als Bytes in Hex" löschen
  ByteHexDaten.Refresh

   
  UpDown1(0).Value = UpDown1(0).Value + 1      ' Anzahl empfangene und angezeigte Frames

'   Nur Test / Debug Code
    
    Select Case Left(strInBuffer, 1)           ' Was für ein Frame haben wir empfangen ?
        
      Case Chr(COM_STX)                        ' Start Text, also wahrscheinlich Frame mit min. 8 Bytes
        Length = Asc(Mid(strShow, COM_POS_VAL_START, 1))   ' Frame - Länge
        
        For iCount = 1 To Length                  ' alle empfangenen Daten auch binär und hex anzeigen, mit 2 Spaces dazwischen
          ByteEmpfDaten.Text = ByteEmpfDaten.Text & Val(Asc(Mid(strShow, iCount, 1))) & Chr(32) & Chr(32)
          ByteHexDaten.Text = ByteHexDaten.Text & Hex(Val(Asc(Mid(strShow, iCount, 1)))) & Chr(32) & Chr(32)  ' geht's auch hex?
          ' rrr wenn da ne NULL vorkommt, ersetzen wir diese für die STRING - ANZEIGE durch einen Platzhalter 1
          If Val(Asc(Mid(strShow, iCount, 1))) = 0 Then
            Mid(strShow, iCount, 1) = Chr(1)
          End If
        
        Next iCount
        
      Case Else                                   ' sonst was, ungültig
        If (SWITCH_DEBUG = True) Then
          MsgBox ("ungültiges Frame")
        End If
        
    End Select
     
    StrEmpfDaten.Text = strShow                  ' Empfangene Daten als String ausgeben
    StrEmpfDaten.Refresh
     
   
   test_string = test_string & Chr(13) & strShow

End Sub

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Datenstrom mit den notwendigen Daten wie Start-Zeichen, Länge, End-Zeichen, Checksumme etc. einpacken         '
'                                                                                                               '
' OP_ID           : Operation, die auf diese Geräte - Eigenschaft angewendet wird: setzen, lesen, usw.          '
' Prop_ID         : Geräte - Eigenschaft: Gerätetyp, Nennspannung der Batterie, Wechsel-Bedingung usw.          '
' InString        : eigentliche Daten des Befehls                                                               '
' yOutStream      : global, kompletter Datenstrom, wie er an die serielle Schnittstelle gehen soll              '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WrapString(yOP_ID As Byte, yProp_ID_Hi As Byte, yProp_ID_Lo As Byte, strInString As String)
  
  Dim iStringLen As Integer                     ' Länge des Werte - Strings (zu bestimmen)
  Dim iCount                                    ' Zählvariable
    
  
  yOutStream(0) = COM_STX                       ' Datenstrom vom PC her beginnt immer mit "Start Text"
  yOutStream(1) = yOP_ID                        ' Op_ID hinzufügen
  yOutStream(2) = yProp_ID_Hi                   ' High Byte der Eigenschaft
  yOutStream(3) = yProp_ID_Lo                   ' Low Byte der Eigenschaft
  
  iStringLen = FnMin(Len(strInString), (COM_MAX_LEN - COM_OVERHEAD))  ' Länge String bestimmen/begrenzen, noch nicht Frame-Länge!
  yOutStream(COM_MAX_LEN) = iStringLen + COM_OVERHEAD                 ' Länge des Frames am Ende speichern, leichter zu verarbeiten
  
  ' Länge des Frames inkl. Overhead ermitteln, an 4.Stelle schreiben
  yOutStream(4) = iStringLen + COM_OVERHEAD
  
  ' Werte-String Byte für Byte in den Out - Stream schreiben, an der richtigen Stelle
  For iCount = 1 To iStringLen
    yOutStream(COM_POS_VAL_START - 1 + iCount) = Asc(Mid(strInString, iCount, 1))
  Next iCount
  
  ' Checksumme (CRC) über alle Bytes bis hierhin ermitteln und schreiben (im Moment nur Quersumme)
  Call CalculateCRC(iStringLen)                 ' Checksumme ermitteln
  
  yOutStream(COM_POS_VAL_START + iStringLen) = yHiCRC               ' High - Byte der Checksumme an drittletzter Stelle
  yOutStream(COM_POS_VAL_START + 1 + iStringLen) = yLoCRC           ' Low - Byte der Checksumme an zweitletzter Stelle
  yOutStream(COM_POS_VAL_START + 2 + iStringLen) = COM_ETX          ' End - Zeichen für Kommunikation

End Sub

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Berechnet die Checksumme (aktuell: Quersumme) aus dem globalen yOutStream                                     '
'                                                                                                               '
' iLenValString   : Länge der zu verpackenden Roh - Daten (ohne Beigemüse)                                      '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub CalculateCRC(iLenValString As Integer)
  
  Dim lLocCRC As Long                     ' VB initialisiert automatisch auf 0
  Dim iCount As Integer
  
  ' Quersumme über alle Bytes ausser den letzten drei bilden (Index beginnt bei 0, deshalb -4)
  For iCount = 0 To (iLenValString + COM_OVERHEAD - 4)
    lLocCRC = lLocCRC + yOutStream(iCount)
  Next iCount

  yHiCRC = FnHiByteOfLong(lLocCRC)                       ' High Byte der Quersumme ermitteln
  yLoCRC = FnLoByteOfLong(lLocCRC)                       ' Low Byte der Quersumme ermitteln

End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Prüft ein übergebenes Frame auf korrekte Länge und korrekte Checksumme; Funktion z.T. analog "CalculateCRC"   '
'                                                                                                               '
' strFrame   : zu prüfendes Empfangs-Frame                                                                      '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function CheckFrameLengthAndCRC(strFrame As String) As Boolean

Dim iCount As Integer                     ' Zähler
Dim lCRC As Long                          ' CRC


If (Asc(Mid(strFrame, COM_POS_VAL_START, 1)) = Len(strFrame)) Then    ' Soll- und Ist-Länge gleich? dann..
  
  For iCount = 1 To (Len(strFrame) - COM_TRAIL_CHARS)                 '..noch Ist-CRC ermitteln und..
    lCRC = lCRC + Asc(Mid(strFrame, iCount, 1))
  Next iCount
  
                                          '..Soll- und Ist-CRC vergleichen
  If lCRC = FnLongOf2Bytes(Asc(Mid(strFrame, Len(strFrame) - 2, 1)), Asc(Mid(strFrame, Len(strFrame) - 1, 1))) Then
    CheckFrameLengthAndCRC = True         ' CRC ok
  Else
    CheckFrameLengthAndCRC = False        ' CRC falsch
  End If                                  ' Ende CRC Vergleich
  
Else
  CheckFrameLengthAndCRC = False
End If                                    ' Ende Längen-Vergleich


End Function


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden des fixfertigen Frames, d.h den in yOutStream enthaltenen Datenstrom Byte für Byte in den Output- ..   '
'..Puffer der RS232 schieben, "kooperativ" warten bis sie abgesetzt sind                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


Private Sub SendFrame()
  
  Dim iCount As Integer                             ' lokale Zählvariable
  
  If MSComm1.PortOpen = True Then                   ' serielle Kommunikation offen?
    
    For iCount = 0 To (yOutStream(COM_MAX_LEN) - 1) ' ganzes Frame in den OutputBuffer
      MSComm1.Output = Chr(yOutStream(iCount))
    Next iCount
    
    Do                                              ' "Halb-Busy-Wait" bis Frame abgesetzt
      ret = DoEvents()                              ' Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until MSComm1.OutBufferCount = 0
  
    Call DispSendFrame                              ' rrr Debug, gesendetes Frame anzeigen
  
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden oder Empfangen der ganzen Kennlinie. Will heissen, vorgegebenen Array mit allen  Parametern ..         '
'..durchzählen (Datenkapsel füllen / auslesen), einen Befehl senden, auf Quittierung warten, ggf. Befehl..      '
'..wiederholen.                                                                                                 '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub SendReceiveProfile(bSet As Boolean)
  
  Dim bErr    As Boolean                              ' lokaler Fehler
  Dim yHi As Byte, yLo As Byte                        ' Hi und Lo Byte der Prop_ID des aktuellen Parameters
  Dim yHiPrev As Byte, yLoPrev As Byte                ' Prop_ID's des vorherigen Par's
  Dim iCount  As Integer                              ' lokale Zählvariable
  Dim lVal As Long                                    ' temporäre Variable für Wert des Parameters
  Dim strTmp  As String                               ' temporärer String
  
  If MSComm1.PortOpen = True Then                     ' serielle Kommunikation offen?
    
  yHiPrev = NOT_PAR                                   ' Am Anfang ist der "vorherige Par" der "NichtParameter" an Stelle 0
  yLoPrev = NOT_PAR                                   '
    
                              ' ------------- alle Parameter durchzählen --------------
  While (GetNextPar(yHiPrev, yLoPrev, yHi, yLo) = True)
    
    lVal = GetDataFromCapsule(yHi, yLo)               ' Wert des aktuellen Parameters holen
    
    strTmp = Chr(FnHiByteOfLong(lVal))                ' High Byte als String
    strTmp = strTmp & Chr(FnLoByteOfLong(lVal))       ' Low Byte als String anhängen
        
    If bSet = True Then                   '--------- Werte Setzen/Kennlinie senden---------- ?
        
      If (GetStatusOfPar(yHi, yLo) = True) Then       ' Darf der Wert des Parameters überschrieben werden?
        Call WrapString(OP_SET, yHi, yLo, strTmp)     ' Frame erzeugen
        Do                                            ' versuche, ein Frame abzusetzen
          Call SendFrame
          Do                                          ' "Halb-Busy-Wait" bis Frame abgesetzt
            ret = DoEvents()                          ' Windows Ereignisse ausführen ("kooperatives Multitasking")
          Loop Until MSComm1.OutBufferCount = 0       ' Stream vollständig abgesetzt
          Call WaitForReply(OP_SET, yHi, yLo, lVal, bErr)     ' Antwort von NLG abfragen / abwarten
        Loop Until (bErr = False)                     ' Frame korrekt abgesetzt? Sonst nochmals versuchen
      End If                                          '
        
    Else                                  '----------- nicht setzen/senden, sondern abfragen --------
          
      strTmp = ""                                     ' beim Abfragen werden keine Daten mitgesendet
      Call WrapString(OP_GET, yHi, yLo, strTmp)
          Do                                          ' versuche, ein Frame abzusetzen
            Call SendFrame
            Do                                        ' "Halb-Busy-Wait" bis Frame abgesetzt
              ret = DoEvents()                        ' Windows Ereignisse ausführen ("kooperatives Multitasking")
            Loop Until MSComm1.OutBufferCount = 0     ' Stream vollständig abgesetzt
            Call WaitForReply(OP_GET, yHi, yLo, lVal, bErr)   ' Antwort von NLG abfragen / abwarten
          Loop Until (bErr = False)                   ' Frame korrekt abgesetzt? Sonst nochmals versuchen
             
    End If                                '----------- Ende setzen / abfragen -------------------------
    
    yHiPrev = yHi
    yLoPrev = yLo
  Wend                                  ' ------------- Ende Parameter durchzählen --------------
  
  Else                                                ' Verbindung nicht offen, also warnen
    MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
  End If

End Sub



' Gesendetes Frame anzeigen, im Wesentlich Debug - Funktion
Private Sub DispSendFrame()
   
  Dim Length As Integer     ' Frame - Länge
  Dim iCount As Integer     ' Zählervariable
  
  ' Initialisierungen
 
  StrSendDaten.Text = ""                        ' Feld "gesendete Daten als String" löschen
  StrSendDaten.Refresh                          '
   
  ByteSendDaten.Text = Empty                    ' Feld "gesendete Daten als Bytes in Dez" löschen
  ByteSendDaten.Refresh
   
  HexSendDaten.Text = Empty                     ' Feld "gesendete Daten als Bytes in Hex" löschen
  HexSendDaten.Refresh
   
  
  For iCount = 0 To (yOutStream(COM_MAX_LEN) - 1)   ' alle empfangenen Daten auch binär anzeigen, mit 2 Spaces dazwischen
    StrSendDaten.Text = StrSendDaten.Text & Chr(FnMax(yOutStream(iCount), 1))     ' Für 0 einen Platzhalter (1) anzeigen
    ByteSendDaten.Text = ByteSendDaten.Text & yOutStream(iCount) & Chr(32) & Chr(32)
    HexSendDaten.Text = HexSendDaten.Text & Hex(yOutStream(iCount)) & Chr(32) & Chr(32)
    UpDown1(3).Value = UpDown1(3).Value + 1         ' Gesendete Bytes / Ereignisse zählen
  Next iCount
      
  UpDown1(2).Value = UpDown1(2).Value + 1           ' Gesendete Frames zählen
    
End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Warten auf passende Antwort zum ausgesendeten Befehl; wenn korrekte Antwort, dann allfälliges Resultat..      '
'..weitergeben, sonst Fehler melden                                                                             '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WaitForReply(yOpID As Byte, yPropIDHi As Byte, yPropIDLo As Byte, lValue As Long, bError As Boolean)
  
  Dim iCount As Integer                             ' lokale Zählvariable
  Dim yAuxByte() As Byte                            ' Zwischenspeicher für Daten
  bError = True                                     ' der wird nur False gesetzt, wenn die Antwort 100%-ig richtig war
  
  
  If MSComm1.PortOpen = True Then                   ' serielle Kommunikation offen?
    
    Do                                              ' "Halb-Busy-Wait" bis Message da
      ret = DoEvents()                              ' Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until g_bMsgReceived = True                ' empfangene Msg pollen
    
    ' Checksumme und Länge des Frames immer prüfen; wenn falsch -> Frame verwerfen
    If (CheckFrameLengthAndCRC(strRecFrameBuf) = True) Then
      
      Select Case yOpID
       
       Case OP_SET                                  ' auf "Set" müssen als Antwort OP_OK, Prop_ID's und Value=ACK folgen
         If (FnGetByteFromFrame(COM_POS_OP_ID, strRecFrameBuf) = OP_OK) Then            ' OP_OK ist ok; sind Prop_ID's auch korrekt?
           If ((FnGetByteFromFrame(COM_POS_PROP_HI, strRecFrameBuf) = yPropIDHi) And (FnGetByteFromFrame(COM_POS_PROP_LO, strRecFrameBuf) = yPropIDLo)) Then    ' Prop_ID Hi und Lo ok
             yAuxByte = FnGetByteFromFrame(COM_POS_VAL_START, strRecFrameBuf)           ' Daten abholen (könnten bei Fehler mehrere sein)
             If (yAuxByte(0) = COM_ACK) Then                                            ' Msg verstanden
               bError = False                                                           ' ok
             End If
           End If
         End If
        
       Case OP_GET                                  ' auf "Get" müssen als Antwort OP_Send, Prop_ID's und  verlangte Daten folgen
         If (FnGetByteFromFrame(COM_POS_OP_ID, strRecFrameBuf) = OP_SEND) Then          ' OP_SEND ist ok; sind Prop_ID's auch korrekt?
           If ((FnGetByteFromFrame(COM_POS_PROP_HI, strRecFrameBuf) = yPropIDHi) And (FnGetByteFromFrame(COM_POS_PROP_LO, strRecFrameBuf) = yPropIDLo)) Then    ' Prop_ID Hi und Lo ok
             yAuxByte = FnGetByteFromFrame(COM_POS_VAL_START, strRecFrameBuf)           ' Daten abholen
             If (UBound(yAuxByte) = LBound(yAuxByte)) Then                              ' Daten konvertieren und weitergeben
               lValue = FnLongOf2Bytes(0, yAuxByte(0))                                  ' Nur Ein Byte -> Hi Byte = 0, Lo Byte = Aux
             Else
               lValue = FnLongOf2Bytes(yAuxByte(0), yAuxByte(1))                        ' Zwei Oder Mehr Bytes -> Hi- und Lo-Byte
             End If
             Call PutDataToCapsule(yPropIDHi, yPropIDLo, lValue)                        ' empfangene Daten in die Kapsel schreiben
             bError = False                                                             ' ok
           End If
         End If
        
        Case Else                                                                       ' Alles Übrige
          If SWITCH_DEBUG = True Then
            MsgBox ("unbekannte OP_ID bei WaitForReply")
          End If
      
      End Select
    
    Else
      bError = True                                 ' Frame nicht ok
    End If                                          ' Ende Frame prüfen
  
  g_bMsgReceived = False                            ' empfangene Msg verarbeitet, also Msg löschen
  
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

End Sub                                             ' Ende WaitForReply


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Ein bestimmtes Byte aus einem Frame zurückgeben. Ausnahme: wenn als StartPosition "Datenfeld" angegeben..     '
'..wird, kommt ein Array mit sovielen Bytes zurück, wie Daten vorhanden sind                                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function FnGetByteFromFrame(yPos As Byte, strFrame As String) As Variant
  
  Dim iCount  As Integer                                                ' Zähl - Variable
  Dim yByte() As Byte
  Dim yLen    As Byte
  
  If (yPos = COM_POS_VAL_START) Then                                    ' Bei den Nutzdaten-Bytes..
    yLen = (Asc(Mid(strFrame, COM_POS_LENGTH + 1, 1)) - COM_OVERHEAD)   ' (keine Rekursionen, darum hier die Länge ausgefieselt)
    ReDim yByte(yLen - 1) As Byte                                       '..Array erweitern
    
    For iCount = 0 To (yLen - 1)                                        ' alle vorhandenen Daten in Array einfüllen
    
      yByte(iCount) = Asc(Mid(strFrame, yPos + iCount + 1, 1))
    Next iCount
    
    FnGetByteFromFrame = yByte()                                        ' verlangte Bytes retournieren
  Else                                                                  ' sonst: genau ein Byte
    FnGetByteFromFrame = Asc(Mid(strFrame, yPos + 1, 1))                ' verlangtes Byte retournieren
  End If
  
End Function
' rrr Debug - Funktion für beliebiges
Private Sub TestBefehl_Click()
  
  Const ARRDIM As Integer = 300
  Dim lDummy As Long, lInputVal As Long
  Dim yDummy1 As Byte, yDummy2 As Byte
  Dim iDummy(ARRDIM) As Integer
  Dim iCount As Integer
  
' Test für private Funktion FindParPos, muss dafür Public gemacht werden
'  For iCount = 0 To ARRDIM
'    iDummy(iCount) = FindParPos(FnHiByteOfLong(CLng(iCount)), FnLoByteOfLong(CLng(iCount)))
'    If (iCount > 25) Then
'      lInputVal = Val(TestBoxA.Text)
'    End If
'  Next iCount
  
  lInputVal = Val(TestBoxA.Text)
  
  'yDummy1 = FnLoByteOfLong(Val(TestBoxA.Text))
  'yDummy2 = FnLoByteOfLong(Val(TestBoxB.Text))
  'lInputVal = FnLongOf2Bytes(yDummy1, yDummy2)
  
  'yDummy1 = FnLoByteOfLong(lInputVal)
  'yDummy2 = FnHiByteOfLong(lInputVal)
 
  'yDummy1 = FnLimit(lInputVal, 125, -125)
End Sub

