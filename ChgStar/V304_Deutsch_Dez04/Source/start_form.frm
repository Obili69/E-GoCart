VERSION 5.00
Begin VB.Form start_form 
   BorderStyle     =   3  'Fester Dialog
   Caption         =   " NLG 5 Software"
   ClientHeight    =   2316
   ClientLeft      =   48
   ClientTop       =   336
   ClientWidth     =   3000
   Icon            =   "start_form.frx":0000
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   Picture         =   "start_form.frx":0CCA
   ScaleHeight     =   2316
   ScaleWidth      =   3000
   ShowInTaskbar   =   0   'False
   StartUpPosition =   1  'Fenstermitte
   Begin VB.CommandButton CmdStartChargeStar 
      Caption         =   "ChargeStar"
      Default         =   -1  'True
      Height          =   492
      Left            =   600
      TabIndex        =   0
      Top             =   120
      Width           =   1812
   End
   Begin VB.CommandButton ComTest 
      Caption         =   "RS232 Test Programm"
      Height          =   492
      Left            =   480
      TabIndex        =   1
      Top             =   840
      Width           =   2052
   End
   Begin VB.CommandButton Raus 
      Cancel          =   -1  'True
      Caption         =   "&EXIT"
      Height          =   492
      Left            =   960
      TabIndex        =   2
      Top             =   1560
      Width           =   1092
   End
   Begin VB.Image nlg5_rot 
      Enabled         =   0   'False
      Height          =   420
      Left            =   3600
      Picture         =   "start_form.frx":7900
      Top             =   120
      Visible         =   0   'False
      Width           =   1800
   End
   Begin VB.Image nlg5_gelb 
      Enabled         =   0   'False
      Height          =   420
      Left            =   3600
      Picture         =   "start_form.frx":920A
      Top             =   600
      Width           =   1800
   End
End
Attribute VB_Name = "start_form"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Dies ist das "Vorfenster", wo man RS232 oder NLG wählen kann   '
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


Private Sub CmdStartChargeStar_Click()
  kennlinie.Show
End Sub


' Button "RS 232 Test" betätigt
Private Sub ComTest_Click()
  SeriellKomm.Show
End Sub


' Der NLG Button wird rot, wenn man drüber fährt
Private Sub nlg5_gelb_MouseMove(Button As Integer, Shift As Integer, x As Single, y As Single)
  nlg5_gelb.Visible = False
  nlg5_rot.Visible = True
End Sub


' Der NLG Button wird gelb, wenn man weg fährt
Private Sub Form_MouseMove(Button As Integer, Shift As Integer, x As Single, y As Single)
  nlg5_gelb.Visible = True
  nlg5_rot.Visible = False
End Sub


'bei Klick auf NLG5 Kennlinie zeigen
Private Sub nlg5_rot_Click()
  Unload Me
  kennlinie.Show
End Sub


' Bye Bye
Private Sub raus_Click()
  Unload Me
  End
End Sub
