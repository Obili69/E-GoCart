VERSION 5.00
Object = "{648A5603-2C6E-101B-82B6-000000000014}#1.1#0"; "MSCOMM32.OCX"
Object = "{86CF1D34-0C5F-11D2-A9FC-0000F8754DA1}#2.0#0"; "Mscomct2.ocx"
Begin VB.Form SeriellKomm 
   Caption         =   "Verbindungs Tester"
   ClientHeight    =   9708
   ClientLeft      =   60
   ClientTop       =   348
   ClientWidth     =   8052
   LinkTopic       =   "Form1"
   ScaleHeight     =   9708
   ScaleWidth      =   8052
   StartUpPosition =   3  'Windows-Standard
   Begin VB.TextBox TxDelay 
      Height          =   288
      Left            =   1680
      TabIndex        =   104
      Text            =   "200"
      Top             =   360
      Width           =   615
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl senden"
      Height          =   495
      Index           =   4
      Left            =   3720
      TabIndex        =   103
      Top             =   2880
      Width           =   735
   End
   Begin VB.TextBox TxtMaxFrameLen 
      Height          =   285
      Left            =   7200
      TabIndex        =   101
      Text            =   "9"
      Top             =   4320
      Width           =   375
   End
   Begin VB.TextBox HexEcho 
      Height          =   375
      Left            =   720
      TabIndex        =   96
      Top             =   9120
      Width           =   6975
   End
   Begin VB.TextBox DezEcho 
      Height          =   375
      Left            =   720
      TabIndex        =   95
      Top             =   8760
      Width           =   6975
   End
   Begin VB.TextBox StrEcho 
      Height          =   375
      Left            =   720
      TabIndex        =   94
      Top             =   8400
      Width           =   6975
   End
   Begin VB.CommandButton Logout 
      Caption         =   "Logout"
      Height          =   495
      Left            =   120
      TabIndex        =   92
      Top             =   720
      Width           =   1095
   End
   Begin VB.CommandButton Login 
      Caption         =   "Login"
      Height          =   495
      Left            =   120
      TabIndex        =   91
      Top             =   120
      Width           =   1095
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   2
      Left            =   6600
      TabIndex        =   90
      Top             =   1680
      Width           =   1095
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   9
      Left            =   7200
      TabIndex        =   83
      Text            =   "3"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   8
      Left            =   6480
      TabIndex        =   81
      Text            =   "10"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   7
      Left            =   6000
      TabIndex        =   79
      Text            =   "13"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   6
      Left            =   5280
      TabIndex        =   78
      Text            =   "101"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   5
      Left            =   4800
      TabIndex        =   76
      Text            =   "108"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      DataField       =   "0"
      Height          =   285
      Index           =   4
      Left            =   4080
      TabIndex        =   75
      Text            =   "105"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   3
      Left            =   3360
      TabIndex        =   73
      Text            =   "102"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   2
      Left            =   2880
      TabIndex        =   72
      Text            =   "111"
      Top             =   3720
      Width           =   375
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   1
      Left            =   2160
      TabIndex        =   70
      Text            =   "114"
      Top             =   3720
      Width           =   375
   End
   Begin VB.CommandButton FolgenderBfSend 
      Caption         =   "&Folgenden Befehl senden"
      Height          =   615
      Left            =   120
      TabIndex        =   69
      Top             =   3600
      Width           =   1092
   End
   Begin VB.TextBox Text2 
      Height          =   285
      Index           =   0
      Left            =   1440
      TabIndex        =   67
      Text            =   "112"
      Top             =   3720
      Width           =   375
   End
   Begin VB.CommandButton ConnectLogin 
      Caption         =   "Verbinden + Einloggen"
      Height          =   495
      Left            =   1440
      TabIndex        =   65
      Top             =   720
      Width           =   1092
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   3
      Left            =   2400
      TabIndex        =   58
      Text            =   "0"
      Top             =   5040
      Width           =   885
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   2
      Left            =   960
      TabIndex        =   57
      Text            =   "0"
      Top             =   5040
      Width           =   885
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   1
      Left            =   2400
      TabIndex        =   55
      Text            =   "0"
      Top             =   6960
      Width           =   915
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   4
      Left            =   5640
      TabIndex        =   54
      Text            =   "24"
      Top             =   3000
      Width           =   615
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   4
      Left            =   4680
      TabIndex        =   53
      Text            =   "69"
      Top             =   3000
      Width           =   615
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   4
      Left            =   6600
      TabIndex        =   52
      Top             =   2880
      Width           =   1095
   End
   Begin VB.CommandButton ClearEcho 
      Caption         =   "Clear Echo"
      Height          =   492
      Left            =   120
      TabIndex        =   51
      Top             =   1320
      Width           =   1092
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   4
      Left            =   6600
      TabIndex        =   50
      Text            =   "6"
      Top             =   3120
      Width           =   1095
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   4
      Left            =   2880
      TabIndex        =   49
      Text            =   "OK Set"
      Top             =   3000
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   3
      Left            =   2880
      TabIndex        =   48
      Text            =   "Error on Set"
      Top             =   2400
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   2
      Left            =   2880
      TabIndex        =   47
      Text            =   "Send Prop"
      Top             =   1800
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   1
      Left            =   2880
      TabIndex        =   46
      Text            =   "Get Prop"
      Top             =   1200
      Width           =   735
   End
   Begin VB.TextBox Text7 
      Height          =   375
      Index           =   0
      Left            =   2880
      TabIndex        =   45
      Text            =   "Set Prop"
      Top             =   480
      Width           =   735
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   3
      Left            =   6600
      TabIndex        =   44
      Text            =   "Text6"
      Top             =   2520
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl senden"
      Height          =   495
      Index           =   3
      Left            =   3720
      TabIndex        =   43
      Top             =   2280
      Width           =   735
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   3
      Left            =   6600
      TabIndex        =   42
      Text            =   "32pq"
      Top             =   2280
      Width           =   1095
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   3
      Left            =   4680
      TabIndex        =   41
      Text            =   "68"
      Top             =   2400
      Width           =   615
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   3
      Left            =   5640
      TabIndex        =   40
      Text            =   "23"
      Top             =   2400
      Width           =   615
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   2
      Left            =   6600
      TabIndex        =   39
      Top             =   1920
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "&Befehl senden"
      Height          =   495
      Index           =   2
      Left            =   3720
      TabIndex        =   38
      Top             =   1680
      Width           =   735
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   2
      Left            =   4680
      TabIndex        =   37
      Text            =   "67"
      Top             =   1800
      Width           =   615
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   2
      Left            =   5640
      TabIndex        =   36
      Text            =   "22"
      Top             =   1800
      Width           =   615
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   1
      Left            =   6600
      TabIndex        =   35
      Top             =   1320
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl senden"
      Height          =   495
      Index           =   1
      Left            =   3720
      TabIndex        =   34
      Top             =   1080
      Width           =   735
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   1
      Left            =   6600
      TabIndex        =   33
      Top             =   1080
      Width           =   1095
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   1
      Left            =   4680
      TabIndex        =   32
      Text            =   "66"
      Top             =   1200
      Width           =   615
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   1
      Left            =   5640
      TabIndex        =   31
      Text            =   "01"
      Top             =   1200
      Width           =   615
   End
   Begin VB.TextBox Text6 
      Height          =   285
      Index           =   0
      Left            =   6600
      TabIndex        =   28
      Text            =   "1500"
      Top             =   600
      Width           =   1095
   End
   Begin VB.TextBox Text1 
      Height          =   285
      Index           =   0
      Left            =   960
      TabIndex        =   27
      Text            =   "0"
      Top             =   6960
      Width           =   915
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   288
      Index           =   0
      Left            =   720
      TabIndex        =   26
      Top             =   6960
      Width           =   276
      _ExtentX        =   466
      _ExtentY        =   508
      _Version        =   393216
      BuddyControl    =   "Text1(0)"
      BuddyDispid     =   196621
      BuddyIndex      =   0
      OrigLeft        =   120
      OrigTop         =   1200
      OrigRight       =   390
      OrigBottom      =   1485
      Max             =   50000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   65547
      Enabled         =   -1  'True
   End
   Begin VB.CommandButton ProfilGet 
      Caption         =   "Ganzes Profil abfragen"
      Height          =   495
      Left            =   1440
      TabIndex        =   25
      Top             =   2520
      Width           =   1095
   End
   Begin VB.CommandButton SendProfile 
      Caption         =   "Ganzes Profil senden"
      Height          =   495
      Left            =   1440
      TabIndex        =   24
      Top             =   1920
      Width           =   1095
   End
   Begin VB.TextBox StrSendDaten 
      Height          =   375
      Left            =   3600
      TabIndex        =   22
      Top             =   5040
      Width           =   4095
   End
   Begin VB.TextBox ByteSendDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   18
      Top             =   5400
      Width           =   6975
   End
   Begin VB.TextBox HexSendDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   17
      Top             =   5760
      Width           =   6975
   End
   Begin VB.TextBox ByteHexDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   13
      Top             =   7680
      Width           =   6975
   End
   Begin VB.TextBox PropText 
      Height          =   375
      Index           =   0
      Left            =   5640
      TabIndex        =   11
      Text            =   "20"
      Top             =   480
      Width           =   615
   End
   Begin VB.TextBox OPText 
      Height          =   375
      Index           =   0
      Left            =   4680
      TabIndex        =   9
      Text            =   "65"
      Top             =   480
      Width           =   615
   End
   Begin VB.TextBox StrEmpfDaten 
      Height          =   375
      Left            =   3600
      TabIndex        =   8
      Top             =   6960
      Width           =   4095
   End
   Begin VB.TextBox ByteEmpfDaten 
      Height          =   375
      Left            =   720
      TabIndex        =   7
      Top             =   7320
      Width           =   6975
   End
   Begin VB.CommandButton test1 
      Caption         =   "&Verbinden"
      Default         =   -1  'True
      Height          =   495
      Left            =   120
      TabIndex        =   4
      Top             =   1920
      Width           =   1095
   End
   Begin VB.TextBox Text4 
      Height          =   285
      Index           =   0
      Left            =   6600
      TabIndex        =   3
      Top             =   360
      Width           =   1095
   End
   Begin VB.CommandButton test2 
      Caption         =   "Befehl senden"
      Height          =   492
      Index           =   0
      Left            =   3720
      TabIndex        =   2
      Top             =   360
      Width           =   735
   End
   Begin VB.CommandButton test3 
      Caption         =   "Verbindung Abbrechen"
      Height          =   495
      Left            =   120
      TabIndex        =   1
      Top             =   2520
      Width           =   1095
   End
   Begin VB.CommandButton Command6 
      Caption         =   "Sende ""Set75V"""
      Height          =   495
      Left            =   1440
      TabIndex        =   0
      Top             =   1320
      Width           =   1095
   End
   Begin MSCommLib.MSComm MSComm1 
      Left            =   360
      Top             =   3000
      _ExtentX        =   995
      _ExtentY        =   995
      _Version        =   393216
      DTREnable       =   -1  'True
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   288
      Index           =   1
      Left            =   2160
      TabIndex        =   56
      Top             =   6960
      Width           =   276
      _ExtentX        =   466
      _ExtentY        =   508
      _Version        =   393216
      BuddyControl    =   "Text1(1)"
      BuddyDispid     =   196621
      BuddyIndex      =   1
      OrigLeft        =   120
      OrigTop         =   1560
      OrigRight       =   390
      OrigBottom      =   1845
      Max             =   100000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   65547
      Enabled         =   -1  'True
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   288
      Index           =   2
      Left            =   720
      TabIndex        =   59
      Top             =   5040
      Width           =   276
      _ExtentX        =   466
      _ExtentY        =   508
      _Version        =   393216
      BuddyControl    =   "Text1(2)"
      BuddyDispid     =   196621
      BuddyIndex      =   2
      OrigLeft        =   120
      OrigTop         =   1200
      OrigRight       =   390
      OrigBottom      =   1485
      Max             =   500000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   0
      Enabled         =   -1  'True
   End
   Begin MSComCtl2.UpDown UpDown1 
      Height          =   288
      Index           =   3
      Left            =   2160
      TabIndex        =   60
      Top             =   5040
      Width           =   276
      _ExtentX        =   466
      _ExtentY        =   508
      _Version        =   393216
      BuddyControl    =   "Text1(3)"
      BuddyDispid     =   196621
      BuddyIndex      =   3
      OrigLeft        =   120
      OrigTop         =   1560
      OrigRight       =   390
      OrigBottom      =   1845
      Max             =   100000
      SyncBuddy       =   -1  'True
      BuddyProperty   =   0
      Enabled         =   -1  'True
   End
   Begin VB.Label Label18 
      Caption         =   "Max Frame Length ->"
      Height          =   252
      Index           =   8
      Left            =   5640
      TabIndex        =   102
      Top             =   4320
      Width           =   1572
   End
   Begin VB.Label Label20 
      Caption         =   "String"
      Height          =   252
      Left            =   240
      TabIndex        =   100
      Top             =   8520
      Width           =   492
   End
   Begin VB.Label Label19 
      Caption         =   "Dez"
      Height          =   252
      Left            =   240
      TabIndex        =   99
      Top             =   8880
      Width           =   372
   End
   Begin VB.Label Label17 
      Caption         =   "Hex"
      Height          =   252
      Left            =   240
      TabIndex        =   98
      Top             =   9240
      Width           =   372
   End
   Begin VB.Label Label4 
      BackColor       =   &H00FFFFFF&
      Caption         =   " Echo (alles)"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.6
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Left            =   720
      TabIndex        =   97
      Top             =   8160
      Width           =   1212
   End
   Begin VB.Label Label18 
      Caption         =   "EOT_Logout"
      Height          =   252
      Index           =   13
      Left            =   2760
      TabIndex        =   93
      Top             =   3480
      Width           =   1212
   End
   Begin VB.Label Label18 
      Caption         =   "2"
      Height          =   252
      Index           =   12
      Left            =   1560
      TabIndex        =   89
      Top             =   3480
      Width           =   132
   End
   Begin VB.Label Label18 
      Caption         =   "10"
      Height          =   252
      Index           =   11
      Left            =   4200
      TabIndex        =   88
      Top             =   3480
      Width           =   372
   End
   Begin VB.Label Label18 
      Caption         =   "3"
      Height          =   252
      Index           =   10
      Left            =   7320
      TabIndex        =   87
      Top             =   3480
      Width           =   132
   End
   Begin VB.Label Label18 
      Caption         =   "EOT"
      Height          =   252
      Index           =   9
      Left            =   5280
      TabIndex        =   86
      Top             =   3480
      Width           =   612
   End
   Begin VB.Label Label18 
      Caption         =   "Set"
      Height          =   252
      Index           =   7
      Left            =   2160
      TabIndex        =   85
      Top             =   3480
      Width           =   372
   End
   Begin VB.Label Label18 
      Caption         =   "Op_ID"
      Height          =   252
      Index           =   6
      Left            =   2040
      TabIndex        =   84
      Top             =   4080
      Width           =   492
   End
   Begin VB.Label Label18 
      Caption         =   "End"
      Height          =   252
      Index           =   5
      Left            =   7200
      TabIndex        =   82
      Top             =   4080
      Width           =   372
   End
   Begin VB.Label Label18 
      Caption         =   "Value 1 / 2"
      Height          =   252
      Index           =   4
      Left            =   4680
      TabIndex        =   80
      Top             =   4080
      Width           =   1092
   End
   Begin VB.Label Label18 
      Caption         =   "CRC HI / Lo"
      Height          =   252
      Index           =   3
      Left            =   6000
      TabIndex        =   77
      Top             =   4080
      Width           =   972
   End
   Begin VB.Label Label18 
      Caption         =   "Length"
      Height          =   252
      Index           =   2
      Left            =   3960
      TabIndex        =   74
      Top             =   4080
      Width           =   492
   End
   Begin VB.Label Label18 
      Caption         =   "Prop HI / Lo"
      Height          =   252
      Index           =   1
      Left            =   2760
      TabIndex        =   71
      Top             =   4080
      Width           =   1092
   End
   Begin VB.Label Label18 
      Caption         =   "Start"
      Height          =   252
      Index           =   0
      Left            =   1440
      TabIndex        =   68
      Top             =   4080
      Width           =   372
   End
   Begin VB.Label lblWaitTimer 
      Caption         =   " Tx Delay (ms)"
      Height          =   252
      Left            =   1440
      TabIndex        =   66
      Top             =   120
      Width           =   1092
   End
   Begin VB.Label Label7 
      Caption         =   "Bytes"
      Height          =   372
      Index           =   3
      Left            =   2400
      TabIndex        =   64
      Top             =   4800
      Width           =   732
   End
   Begin VB.Label Label7 
      Caption         =   "Frames"
      Height          =   252
      Index           =   2
      Left            =   1080
      TabIndex        =   63
      Top             =   4800
      Width           =   732
   End
   Begin VB.Label Label7 
      Caption         =   "Bytes"
      Height          =   252
      Index           =   1
      Left            =   2520
      TabIndex        =   62
      Top             =   6720
      Width           =   732
   End
   Begin VB.Label Label7 
      Caption         =   "Frames"
      Height          =   252
      Index           =   0
      Left            =   960
      TabIndex        =   61
      Top             =   6720
      Width           =   732
   End
   Begin VB.Label Label15 
      Caption         =   "Dez - Wert"
      Height          =   255
      Index           =   0
      Left            =   6720
      TabIndex        =   30
      Top             =   840
      Width           =   975
   End
   Begin VB.Label Label3 
      Caption         =   "oder"
      Height          =   255
      Index           =   0
      Left            =   7680
      TabIndex        =   29
      Top             =   480
      Width           =   375
   End
   Begin VB.Label Label14 
      Caption         =   "String"
      Height          =   252
      Left            =   4080
      TabIndex        =   23
      Top             =   4800
      Width           =   492
   End
   Begin VB.Label Label13 
      BackColor       =   &H00FFFFFF&
      Caption         =   "  Gesendete Daten"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.6
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Left            =   720
      TabIndex        =   21
      Top             =   4440
      Width           =   1932
   End
   Begin VB.Label Label12 
      Caption         =   "Dez"
      Height          =   252
      Left            =   240
      TabIndex        =   20
      Top             =   5520
      Width           =   372
   End
   Begin VB.Label Label11 
      Caption         =   "Hex"
      Height          =   252
      Left            =   240
      TabIndex        =   19
      Top             =   5880
      Width           =   372
   End
   Begin VB.Label Label10 
      Caption         =   "Hex"
      Height          =   252
      Left            =   240
      TabIndex        =   16
      Top             =   7800
      Width           =   372
   End
   Begin VB.Label Label9 
      Caption         =   "Dez"
      Height          =   252
      Left            =   240
      TabIndex        =   15
      Top             =   7440
      Width           =   372
   End
   Begin VB.Label Label8 
      Caption         =   "String"
      Height          =   252
      Left            =   3840
      TabIndex        =   14
      Top             =   6720
      Width           =   492
   End
   Begin VB.Label Label2 
      Caption         =   "Prop_ID (dez)"
      Height          =   255
      Index           =   0
      Left            =   5520
      TabIndex        =   12
      Top             =   120
      Width           =   1095
   End
   Begin VB.Label Label1 
      Caption         =   "OP_ID (dez)"
      Height          =   255
      Index           =   0
      Left            =   4560
      TabIndex        =   10
      Top             =   120
      Width           =   975
   End
   Begin VB.Label LabelEmpfDaten 
      BackColor       =   &H00FFFFFF&
      Caption         =   "  Empfangene Daten (nur ganze Frames)"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.6
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Left            =   720
      TabIndex        =   6
      Top             =   6360
      Width           =   3732
   End
   Begin VB.Label Label5 
      Caption         =   "Datenstring"
      Height          =   255
      Index           =   0
      Left            =   6720
      TabIndex        =   5
      Top             =   120
      Width           =   855
   End
End
Attribute VB_Name = "SeriellKomm"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / SeriellKomm                                                             '
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
'             Feb. '02  / Beat G.     / Anpassungen für SHC1-B / neue NLG-Par's                                 '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
' MSComm1_OnComm                      / "Interrupt" - Funktion, empfängt und puffert ankommende Zeichen         '
' CalculateCRC                        / Checksumme des zu sendenden Datenstroms berechnen                       '
' FnGetByteFromFrame                  / ein bestimmtes Byte aus einem Frame holen                               '
' CheckFrameLengthAndCRC              / Länge und Checksumme eines Frames auf Korrektheit prüfen                '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'

Option Explicit

Dim fenster As Boolean
Dim envoyer As Boolean
Dim test_string As String

Dim bBinNLG As Boolean                  ' bin ich das NLG?

'
'
' Achtung:  Mangels sinnvoller Alternativen wurde die Datenhaltung des Sende - Streams mit einem globalen Array
'           yOutStream realisiert; dito CRC
' Und es geht noch weiter: da das Kommunikationsobjekt am entsprechenden Formular (z.B. Kennlinien - Programm)..
'..hängt, wird die Empfangsfunktion "MSComm1_OnComm" auch dort platziert.
'
'


' diese voll globalen Variablen sollten eigentlich einigermassen privat oder wenigstens modulglobal werden
Dim yOutStream(COM_MAX_LEN) As Byte      ' total globaler Array für den Out - Stream
Dim yHiCRC As Byte, yLoCRC As Byte       ' total globale Variablen für CRC Hi und Lo Byte

' "Schnittstelle" zur Empfangs - Funktion (Telefönli-Objekt, Comm1)
Dim strInBuffer As String                ' String variabler Länge, hier kommen die empfangenen Daten rein
Dim strRecFrameBuf As String             ' empfangenes, komplettes Frame zwischenspeichern zum Verarbeiten

Dim ret As Integer                          ' nur für Debug







'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Verbindung zum NLG aufbauen bzw. abbrechen, login, steuerbefehle absetzen etc. Das eigentliche Öffnen des..   '
'..Comm - Ports muss beim Objekt selber geschehen, also im SeriellKomm Programm                                '
'                                                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Sub ConnectNLG()
  
  Dim lVal    As Long                                   ' Rückgabewert
  Dim iRetry  As Integer                                ' Sende - Zähler
  Dim yHi     As Byte                                   ' Hi - Byte der Property
  Dim yLo     As Byte                                   ' Lo - Byte der Property
  Dim strAux  As String                                 ' Hilfs - String
  Dim bErr    As Boolean                                ' Fehler in der Kommunikation
  Dim yAuxStream(COM_MAX_LEN) As Byte
  Dim iLoop As Integer
                                                        ' Login - String erstellen
  
'  yOutStream(0) = 112       'p
'  yOutStream(1) = 114       'r
'  yOutStream(2) = 111       'o
'  yOutStream(3) = 102       'f
'  yOutStream(4) = 105       'i
'  yOutStream(5) = 108       'l
'  yOutStream(6) = 101       'e
'  yOutStream(7) = 13        'cr
'  yOutStream(8) = 10        'lf
'  yOutStream(COM_MAX_LEN) = 9
'
'  Call SendFrame                                      ' Login - String senden
'
  
    
    yAuxStream(0) = 112       'p
    yAuxStream(1) = 114       'r
    yAuxStream(2) = 111       'o
    yAuxStream(3) = 102       'f
    yAuxStream(4) = 105       'i
    yAuxStream(5) = 108       'l
    yAuxStream(6) = 101       'e
    yAuxStream(7) = 13        'cr
    yAuxStream(8) = 10        'lf
    
    For iLoop = 0 To 8                                  ' bg025 login string nicht "am Stück" senden
      yOutStream(0) = yAuxStream(iLoop)
      yOutStream(COM_MAX_LEN) = 1
      Call SendFrame                                    ' Login - String zeichenweise senden
      WaitForEcho (Chr(yOutStream(0)))
    Next iLoop                                          ' end bg025
    
  
  
  
  ' allfälligen Ladevorgang stoppen
  bErr = True
  iRetry = 0                                          ' Sende - Zähler initialisieren

' prop "Name" abfragen:
    yHi = FnHiByteOfLong(INF_NLG_TYP_51X)             ' Hi - Byte
    yLo = FnLoByteOfLong(INF_NLG_TYP_51X)             ' Lo - Byte
    Call WrapString(OP_GET, yHi, yLo, "")             ' Abfrage, keine Daten mitsenden
' end ddd

  While ((bErr = True) And (iRetry < ERR_RETRY))      ' ggf. mehrere Versuche
    Call SendFrame                                    ' Stop - Befehl senden
    Call WaitForReply(OP_GET, yHi, yLo, lVal, bErr)   ' Antwort von NLG abfragen / abwarten
    iRetry = iRetry + 1                               ' Anz. Sendeversuche inkrementieren
  Wend
  
  strAux = strAux & Chr$(69)                          ' Dummy Instruktion

End Sub                                               ' Ende ConnectNLG





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden oder Empfangen der ganzen Kennlinie. Will heissen, vorgegebenen Array mit allen  Parametern ..         '
'..durchzählen (Datenkapsel füllen / auslesen), einen Befehl senden, auf Quittierung warten, ggf. Befehl..      '
'..wiederholen.                                                                                                 '
'                                                                                                               '
' bSet            : Flag für Senden (True) / Abfragen (False) der Kennlinie                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub SendReceiveProfile(bSet As Boolean)
  
  Dim bErr    As Boolean                              ' lokaler Fehler
  Dim yHi As Byte, yLo As Byte                        ' Hi und Lo Byte der Prop_ID des aktuellen Parameters
  Dim yHiPrev As Byte, yLoPrev As Byte                ' Prop_ID's des vorherigen Par's
  Dim iCount  As Integer                              ' lokale Zählvariable
  Dim lVal As Long                                    ' temporäre Variable für Wert des Parameters
  Dim strTmp  As String                               ' temporärer String
  Dim iErr  As Integer                                ' Fehler - Zähler, um 'Hänger' zu detektieren
  
  If MSComm1.PortOpen = True Then                     ' serielle Kommunikation offen?
    
  yHiPrev = NOT_PAR                                   ' Am Anfang ist der "vorherige Par" der "NichtParameter" an Stelle 0
  yLoPrev = NOT_PAR                                   '
    
                              ' ------------- alle Parameter durchzählen --------------
  While (GetNextPar(yHiPrev, yLoPrev, yHi, yLo) = True)
    
    iErr = 0                                          ' Fehler - Zähler reseten
    
    lVal = GetDataFromCapsule(yHi, yLo)               ' Wert des aktuellen Parameters holen
    
    strTmp = Chr(FnHiByteOfLong(lVal))                ' High Byte als String
    strTmp = strTmp & Chr(FnLoByteOfLong(lVal))       ' Low Byte als String anhängen
        
    If bSet = True Then                   '--------- Werte Setzen/Kennlinie senden---------- ?
        
      If (GetStatusOfPar(yHi, yLo) = True) Then       ' Darf der Wert des Parameters überschrieben werden?
        Call WrapString(OP_SET, yHi, yLo, strTmp)     ' Frame erzeugen
        Do                                            ' versuche, ein Frame abzusetzen
          Call SendFrame
          Do                                          ' "Halb-Busy-Wait" bis Frame abgesetzt
            ret = DoEvents()                          ' Abbruch? Windows Ereignisse ausführen ("kooperatives Multitasking")
          Loop Until MSComm1.OutBufferCount = 0       ' Stream vollständig abgesetzt
          Call WaitForReply(OP_SET, yHi, yLo, lVal, bErr)     ' Antwort von NLG abfragen / abwarten
          If (bErr = True) Then                       ' Fehler-Zähler
            iErr = iErr + 1
          End If
        Loop Until ((bErr = False) Or (iErr > ERR_RETRY))     ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
        If (iErr > ERR_RETRY) Then
          If (SWITCH_DEBUG = True) Then
            MsgBox ("Parameter setzen / lesen nicht möglich")
          End If
        End If
      End If                                          '
        
    Else                                  '----------- nicht setzen/senden, sondern abfragen --------
          
      strTmp = ""                                     ' beim Abfragen werden keine Daten mitgesendet
      Call WrapString(OP_GET, yHi, yLo, strTmp)
          Do                                          ' versuche, ein Frame abzusetzen
            Call SendFrame
            Do                                        ' "Halb-Busy-Wait" bis Frame abgesetzt
              ret = DoEvents()                        ' Abbruch? Windows Ereignisse ausführen ("kooperatives Multitasking")
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

End Sub                             'Ende SendReceiveProfile






'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden des fixfertigen Frames, d.h den in yOutStream enthaltenen Datenstrom Byte für Byte in den Output- ..   '
'..Puffer der RS232 schieben, "kooperativ" warten bis sie abgesetzt sind                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub SendFrame()
  
  Dim iCount As Integer                             ' lokale Zählvariable
  Dim fWaitTimer As Single
  Dim fTimeStamp As Single                          ' Zeitstempel für's Warten
  
  
  fWaitTimer = Val(TxDelay.Text) / 1000         ' rrr ddd grundsätzlich warten, vor jedem Zeichen
    
  
  If MSComm1.PortOpen = True Then                   ' serielle Kommunikation offen?
    
    For iCount = 0 To (yOutStream(COM_MAX_LEN) - 1) ' ganzes Frame in den OutputBuffer
  
      fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
      Do                                                ' "Halb-Busy-Wait" bis Message da
        ret = DoEvents()                                ' Abbruch-Bed vorh.! Windows Ereignisse ausführen ("kooperatives Multitasking")
      Loop Until ((Timer - fTimeStamp) > fWaitTimer)    '  Timeout prüfen
      
      MSComm1.Output = Chr(yOutStream(iCount))
    Next iCount
    
    Do                                              ' "Halb-Busy-Wait" bis Frame abgesetzt
      ret = DoEvents()                              ' Abbruch? Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until MSComm1.OutBufferCount = 0
  
  Call WaitForSent                                  ' warten, bis Frame abgesetzt oder Timeout
  
  Call DispSendFrame                              ' ddd Debug, gesendetes Frame anzeigen
  
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If

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
' Warten auf passende Antwort zum ausgesendeten Befehl; wenn korrekte Antwort, dann allfälliges Resultat..      '
'..weitergeben, sonst Fehler melden                                                                             '
'                                                                                                               '
' yOpID           : Operation, die auf diese Geräte - Eigenschaft angewendet wird: setzen, lesen, usw.          '
' yPropIDHi/Lo    : Geräte - Eigenschaft: Gerätetyp, Nennspannung der Batterie, Wechsel-Bedingung usw.          '
' lValue          : allfällig zurückgegebener Wert der Eigenschaft                                              '
' bError          : Flag, welches meldet, ob ein Fehler auftrat (True) oder korrekte Antwort kam (False)        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WaitForReply(yOpID As Byte, yPropIDHi As Byte, yPropIDLo As Byte, lValue As Long, bError As Boolean)
  
  Dim iCount As Integer                             ' lokale Zählvariable
  Dim yAuxByte() As Byte                            ' Zwischenspeicher für Daten
  Dim fTimeStamp As Single                          ' Zeitstempel für's Warten
  Dim bTimeOut As Boolean                           ' Flag für Timeout
  
  
  fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
  bError = True                                     ' der wird nur False gesetzt, wenn die Antwort 100%-ig richtig war
  bTimeOut = False                                  ' muss man wohl jedes mal initialisieren
 
  If (MSComm1.PortOpen = True) Then                   ' serielle Kommunikation offen?
    
    Do                                              ' "Halb-Busy-Wait" bis Message da
      If ((Timer - fTimeStamp) > COM_TIMEOUT) Then
        bTimeOut = True                             ' keine Antwort innerhalb nützlicher Frist erhalten
      End If
      ret = DoEvents()                              ' Abbruch-Bed vorh.! Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until ((g_bMsgReceived = True) Or (bTimeOut = True)) ' empfangene Msg pollen, Timeout prüfen
    
    ' Antwort ist innerhalb der erwarteten Frist eingetroffen
    If (bTimeOut = False) Then
      
      ' Checksumme und Länge des Frames immer prüfen; wenn falsch -> Frame verwerfen
      If (CheckFrameLengthAndCRC(strRecFrameBuf) = True) Then
        
        Select Case yOpID
         
          Case OP_SET                                  ' auf "Set" müssen als Antwort OP_OK und Prop_ID's folgen
            If (FnGetByteFromFrame(COM_POS_OP_ID, strRecFrameBuf) = OP_OK) Then             ' OP_OK ist ok; sind Prop_ID's auch korrekt?
              If ((FnGetByteFromFrame(COM_POS_PROP_HI, strRecFrameBuf) = yPropIDHi) And (FnGetByteFromFrame(COM_POS_PROP_LO, strRecFrameBuf) = yPropIDLo)) Then    ' Prop_ID Hi und Lo ok
                yAuxByte = FnGetByteFromFrame(COM_POS_VAL_START, strRecFrameBuf)            ' Daten abholen (könnten bei Fehler mehrere sein)
                If (FnLongOf2Bytes(yPropIDHi, yPropIDLo) < DIRECT_CONTROL_CMDS) Then        ' "normale" Properties
                  If (yAuxByte(0) = COM_ACK) Then                                           ' Msg verstanden, bei "normaler" Property
                    bError = False                                                          ' ok
                  End If
                Else                                                                        ' Direct Control Command
                  Select Case (yAuxByte(0))
                    Case COM_STOP_CHG                                                       ' Ladevorgang gestoppt
                      bError = False                                                        ' diese Antwort ist in Ordnung
                    Case COM_START_CHG                                                      ' Ladevorgang gestartet
                      bError = False                                                        ' diese Antwort ist in Ordnung
                  End Select
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
    
    Else                                            ' keine rechtzeitige Antwort eingetroffen
      If SWITCH_DEBUG = True Then
        MsgBox ("Timeout bei Warten auf Antwort")
      End If
    End If                                          ' Ende rechtzeitige Antwort eingetroffen
    
  g_bMsgReceived = False                            ' empfangene Msg verarbeitet, also Msg löschen
  
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

End Sub                                             ' Ende WaitForReply




Private Sub ClearEcho_Click()
   StrEcho.Text = ""                             ' Feld "Echo als String" löschen
   StrEcho.Refresh

   DezEcho.Text = Empty                          ' Feld "Echo als Dez" löschen
   DezEcho.Refresh

   HexEcho.Text = Empty                          ' Feld "Echo als Hex" löschen
   HexEcho.Refresh
   
End Sub


Private Sub Logout_Click()
  
  Call WrapString(OP_SET, 240, 1, (Chr(0) & Chr(4)))     ' Logout
  Call SendFrame                                                ' soeben erstelltes Frame senden
  
End Sub

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Empfängt ankommende Zeichen, schiebt diese in den (temporären) Buffer und erhöht dessen Zähler um 1; wenn..   '
'.. gültiges Frame erkannt, d.h. Länge und Overhead stimmen, wird die Auswertung aufgerufen                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Static Sub MSComm1_OnComm()

Dim strEmpfString     As String                       ' Empfangs - String, wird aus den einzelnen Zeichen gebastelt
Static bBufOverflow   As Boolean                      ' zeigt an, wenn ein Überlauf im Empfangspuffer vorgekommen ist
Static iEchoChars       As Integer
Dim strAux As String

  'Debug.Print MSComm1.CommEvent                      ' braucht's nicht "sende das Kommunikatons - Ereignis ans Direktfenster"
    
    
    
  If MSComm1.CommEvent = comEvReceive Then            ' etwas empfangen
    
    strEmpfString = ""
    strEmpfString = MSComm1.Input                     ' Eingangspuffer holen / löschen
    UpDown1(1).Value = UpDown1(1).Value + 1           ' Empfangene Bytes / Ereignisse zählen
    
 
    'String Echo, "Klartext"
    If Val(Asc(strEmpfString)) = 0 Then
      strAux = Chr(1) & " "
    Else
      strAux = strEmpfString & " "
    End If

    StrEcho.Text = StrEcho.Text & strAux                                ' Zeichen Echo
    StrEcho.Refresh
    
    DezEcho.Text = DezEcho.Text & Val(Asc(strEmpfString)) & "  "        ' Dez Echo
    DezEcho.Refresh
    
    HexEcho.Text = HexEcho.Text & Hex(Val(Asc(strEmpfString))) & "  "   ' Hex Echo
    HexEcho.Refresh
    
    
    
    
    strInBuffer = strInBuffer & strEmpfString         ' empfangenen String an Puffer anhängen
    
    ' Wenn Länge des Strings kleiner gleich max. Länge, ist's ok; sonst: Fehler
    If Len(strInBuffer) <= COM_MAX_LEN Then
      If (Len(strInBuffer) > COM_POS_VAL_START) Then  ' ab dieser Länge könnte es ein korrektes Frame sein
        Call Display_String(strInBuffer)       ' ddd für Debug vorgezogen: empfangenen String anzeigen
        If Asc(Right(strInBuffer, 1)) = COM_ETX Then  ' auf "ETX" (Ascii 3) triggern, also Ende Frame
          ' Wenn Länge des Strings wie angekündigt, dann..
          If Asc(Mid(strInBuffer, COM_POS_LENGTH + 1, 1)) = Len(strInBuffer) Then
            UpDown1(0).Value = UpDown1(0).Value + 1      ' Anzahl empfangene und angezeigte Frames
'           If CheckNLG.Value = True Then
            strRecFrameBuf = strInBuffer              ' empfangenes, komplettes Frame puffern (noch nicht auf Korrektheit geprüft)
'            Call Display_String(strRecFrameBuf)       ' empfangenen String anzeigen
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
    
  Else                                                ' nicht empfangen, sondern sonst was
    If (0) Then
    Else
    End If
  End If
    
End Sub                     ' Ende MSComm1_OnComm





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Warten bis ein Zeichen eintrifft. Wird nur für den Login gebraucht, weil da u.U. das NLG nur sehr langsam..   '
'..kommuniziert, bis die Interrupts abgeschaltet sind. Wenn entweder richtiges Zeichen kommt, oder die Zeit..   '
'..abgelaufen ist, geht's weiter                                                                                '
'                                                                                                               '
' strChar           : zu vergleichendes Zeichen                                                                 '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WaitForEcho(strChar As String)
  
  Dim fTimeStamp As Single                          ' Zeitstempel für's Warten
  Dim bTimeOut As Boolean                           ' Flag für Timeout
  
  
  fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
  bTimeOut = False                                  ' muss man wohl jedes mal initialisieren
  
  If (MSComm1.PortOpen = True) Then                 ' serielle Kommunikation offen?
    
    Do                                              ' "Halb-Busy-Wait" bis Message abgesetzt
      If ((Timer - fTimeStamp) > LOGIN_WAIT) Then
        bTimeOut = True                             ' keine Antwort innerhalb nützlicher Frist erhalten
      End If
      ret = DoEvents()                              ' Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until ((g_strEmpfString = strChar) Or (bTimeOut = True))  ' Echo da oder Timeout, weiter
    
'    If (bTimeOut = True) Then                       ' Message konnte nicht in vernünftiger Zeit abgesetzt werden
'      If SWITCH_DEBUG = True Then
'        MsgBox ("Kein Echo erhalten")
'      End If
'    End If                                          ' Ende rechtzeitige Antwort eingetroffen
 
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Bei WaitForEcho besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

End Sub                                             ' Ende WaitForEcho




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

iCount = 0                                                      ' Dummy

For iCount = 0 To (FnMin(Val(Text2(4).Text), Val(TxtMaxFrameLen.Text)) - 1)     ' Stringlänge bestimmen
  yOutStream(iCount) = Val(Text2(iCount).Text)                  ' Datenstrom mit gewünschten Bytes füllen
Next iCount

yOutStream(COM_MAX_LEN) = iCount

Call SendFrame                                                  ' Frame versenden

End Sub

' Initialisierung
Private Sub Form_Load()
  strInBuffer = ""
  fenster = True
  envoyer = False
End Sub
' debug, Login String ans NLG senden
Private Sub Login_Click()
 
'  yOutStream(0) = 112       'p
'  yOutStream(1) = 114       'r
'  yOutStream(2) = 111       'o
'  yOutStream(3) = 102       'f
'  yOutStream(4) = 105       'i
'  yOutStream(5) = 108       'l
'  yOutStream(6) = 101       'e
'  yOutStream(7) = 13        'cr
'  yOutStream(8) = 10        'lf
'  yOutStream(COM_MAX_LEN) = 9
'
'  Call SendFrame

Call ConnectNLG

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

   

'   Nur Test / Debug Code
    
    Select Case FnMin(Left(strShow, 1), Len(strShow))           ' Was für ein Frame haben wir empfangen ?
        
      Case Chr(COM_STX)                        ' Start Text, also wahrscheinlich Frame mit min. 8 Bytes
        Length = Asc(Mid(strShow, COM_POS_VAL_START, 1))   ' Frame - Länge
        
        For iCount = 1 To Length                  ' alle empfangenen Daten auch binär und hex anzeigen, mit 2 Spaces dazwischen
          ByteEmpfDaten.Text = ByteEmpfDaten.Text & Val(Asc(Mid(strShow, iCount, 1))) & Chr(32) & Chr(32)
          ByteHexDaten.Text = ByteHexDaten.Text & Hex(Val(Asc(Mid(strShow, iCount, 1)))) & Chr(32) & Chr(32)  ' geht's auch hex?
          ' ddd wenn da ne NULL vorkommt, ersetzen wir diese für die STRING - ANZEIGE durch einen Platzhalter 1
          If Val(Asc(Mid(strShow, iCount, 1))) = 0 Then
            Mid(strShow, iCount, 1) = Chr(1)
          End If
        
        Next iCount
        
      Case Else                                   ' sonst was, ungültig
        If (SWITCH_DEBUG = True) Then
'          MsgBox ("ungültiges Frame")
          For iCount = 1 To Len(strInBuffer)                 ' alle empfangenen Daten auch binär und hex anzeigen, mit 2 Spaces dazwischen
            ByteEmpfDaten.Text = ByteEmpfDaten.Text & Val(Asc(Mid(strShow, iCount, 1))) & Chr(32) & Chr(32)
            ByteHexDaten.Text = ByteHexDaten.Text & Hex(Val(Asc(Mid(strShow, iCount, 1)))) & Chr(32) & Chr(32)  ' geht's auch hex?
          ' ddd wenn da ne NULL vorkommt, ersetzen wir diese für die STRING - ANZEIGE durch einen Platzhalter 1
            If Val(Asc(Mid(strShow, iCount, 1))) = 0 Then
              Mid(strShow, iCount, 1) = Chr(1)
            End If
          Next iCount

        End If
        
    End Select
     
    StrEmpfDaten.Text = strShow                  ' Empfangene Daten als String ausgeben
    StrEmpfDaten.Refresh
     
   
   test_string = test_string & Chr(13) & strShow

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
' Ein bestimmtes Byte aus einem Frame zurückgeben. Ausnahme: wenn als StartPosition "Datenfeld" angegeben..     '
'..wird, kommt ein Array mit sovielen Bytes zurück, wie Daten vorhanden sind                                    '
'                                                                                                               '
' yPos            : Position im Frame des gewünschten Bytes                                                     '
' strFrame        : auszuwertendes Frame als String                                                             '
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



' verbinden und Login
Private Sub ConnectLogin_Click()
  
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
  
  
    If MSComm1.PortOpen = False Then                    ' serielle Schnittstelle öffnen
      
      MSComm1.CommPort = 1
      MSComm1.Settings = "19200,n,8,1"
      MSComm1.InputLen = 1
      MSComm1.RThreshold = 1
      MSComm1.SThreshold = 1
      MSComm1.PortOpen = True
  
      Call ConnectNLG
      
    Else
      If (SWITCH_DEBUG = True) Then
        MsgBox ("Es besteht schon eine Verbindung")
      End If
    End If
    
      
End Sub






'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Warten bis ein Datenpaket im Comm - Buffer vollständig abgesetzt ist. Es ist aber kein "echtes" Busy Wait,..  '
'..das Lose Doze Messaging System läuft trotzdem weiter; aber die Applikation muss hier solange warten, bis..   '
'..sie weiterfahren darf                                                                                        '
'                                                                                                               '
' bDone           : allfällige Wartezeit ist abgelaufen, es kann weitergehn                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WaitForSent()
  
  Dim fTimeStamp As Single                          ' Zeitstempel für's Warten
  Dim bTimeOut As Boolean                           ' Flag für Timeout
  Dim iWaitTimer As Integer                         ' Wartezeit in ms
  
  
  fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
  bTimeOut = False                                  ' muss man wohl jedes mal initialisieren
  
  If (MSComm1.PortOpen = True) Then                 ' serielle Kommunikation offen?
    
    Do                                              ' "Halb-Busy-Wait" bis Message abgesetzt
      If ((Timer - fTimeStamp) > COM_TIMEOUT) Then
        bTimeOut = True                             ' keine Antwort innerhalb nützlicher Frist erhalten
      End If
      ret = DoEvents()                              ' Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until ((MSComm1.OutBufferCount = BUF_EMPTY) Or (bTimeOut = True))  ' Message abgesetzt oder Timeout erreicht, abbrechen
    
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Bei WaitForSent besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

End Sub                                             ' Ende WaitForSent




