VERSION 5.00
Object = "{831FDD16-0C5C-11D2-A9FC-0000F8754DA1}#2.0#0"; "MSCOMCTL.OCX"
Begin VB.Form UpDownLoad 
   BorderStyle     =   1  'Fest Einfach
   Caption         =   "Daten übertragen"
   ClientHeight    =   1884
   ClientLeft      =   2760
   ClientTop       =   3756
   ClientWidth     =   4188
   ControlBox      =   0   'False
   Icon            =   "UpDownLoadData.frx":0000
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   1884
   ScaleWidth      =   4188
   Tag             =   "False"
   Begin MSComctlLib.ProgressBar ProgressBar1 
      Height          =   375
      Left            =   240
      TabIndex        =   1
      Top             =   720
      Width           =   3735
      _ExtentX        =   6583
      _ExtentY        =   656
      _Version        =   393216
      Appearance      =   1
   End
   Begin VB.CommandButton CancelButton 
      Caption         =   "Abbrechen"
      Height          =   375
      Left            =   1440
      TabIndex        =   0
      Top             =   1320
      Width           =   1215
   End
   Begin VB.Label lblWait 
      Caption         =   "Daten werden übertragen......"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.6
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Left            =   840
      TabIndex        =   2
      Tag             =   "1"
      Top             =   240
      Width           =   2535
   End
End
Attribute VB_Name = "UpDownLoad"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Name                            UpDownLoad                                                              '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Beschreibung                    Das Fensterchen, welches während des uploads oder downloads eines ..    '
'                                       ..NLG-Datensatzes den Fortschritt anzeigt und einen Abbruch ermöglicht. '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Was       / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / März '01  / Beat G.     /                                                                         '
' V2.0      / April '03 / Beat G.     / mehrsprachige Version (Deutsch / Englisch)                              '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
'

Option Explicit



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Übertragungs-Fensterchen laden, Titel und "bin beschäftigt"-Text anzeigen                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


Private Sub Form_Load()

UpDownLoad.Caption = LoadResString(STR_ID_RTXBOX_TITLE + g_lLocaleOffset)
UpDownLoad.lblWait = LoadResString(STR_ID_RTXBOX_WAIT_MSG + g_lLocaleOffset)

End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Cancel - Button geklickt, also Fortschritts - Anzeige - Dialog schliessen und Hauptprogramm über gewünschten..'
'..Abbruch der Datenübertragung informieren                                                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub CancelButton_Click()

  Dim iDum1 As Integer, iDum2 As Integer        ' Dummies
  
'  g_bMsgStopTransmitData = True                ' Datenübertragung abbrechen
  Call Form_QueryUnload(iDum1, iDum2)
  Unload Me                                     ' Fortschritts - Dialog abschiessen
End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fortschritts-Dialog sonstwie geschlossen -> tue dasselbe wie CancelButton_Click, aber lass das schliessen..   '
'..beiseite, das macht's eh schon selber                                                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_QueryUnload(Cancel As Integer, UnloadMode As Integer)
  
  g_bMsgStopTransmitData = True                 ' Datenübertragung abbrechen

End Sub

