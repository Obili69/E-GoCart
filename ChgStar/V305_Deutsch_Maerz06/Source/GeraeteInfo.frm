VERSION 5.00
Begin VB.Form GeraeteInfo 
   BorderStyle     =   1  'Fest Einfach
   Caption         =   "Geräte - Info"
   ClientHeight    =   2844
   ClientLeft      =   2760
   ClientTop       =   3756
   ClientWidth     =   3828
   ControlBox      =   0   'False
   Icon            =   "GeraeteInfo.frx":0000
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   2844
   ScaleWidth      =   3828
   Begin VB.TextBox TextFirmwareVers 
      Alignment       =   1  'Rechts
      BeginProperty Font 
         Name            =   "Arial"
         Size            =   10.2
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   360
      Left            =   2160
      Locked          =   -1  'True
      TabIndex        =   8
      Top             =   1740
      Width           =   1452
   End
   Begin VB.TextBox TextProdDat 
      Alignment       =   1  'Rechts
      BeginProperty Font 
         Name            =   "Arial"
         Size            =   10.2
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   360
      Left            =   2160
      Locked          =   -1  'True
      TabIndex        =   6
      Top             =   1260
      Width           =   1452
   End
   Begin VB.TextBox TextSerNr 
      Alignment       =   1  'Rechts
      BeginProperty Font 
         Name            =   "Arial"
         Size            =   10.2
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   360
      Left            =   2160
      Locked          =   -1  'True
      TabIndex        =   4
      Top             =   780
      Width           =   1452
   End
   Begin VB.TextBox TextGerTyp 
      Alignment       =   1  'Rechts
      BeginProperty Font 
         Name            =   "Arial"
         Size            =   10.2
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   360
      Left            =   2160
      Locked          =   -1  'True
      TabIndex        =   2
      Top             =   300
      Width           =   1455
   End
   Begin VB.CommandButton OKButton 
      Caption         =   "OK"
      Height          =   375
      Left            =   1320
      TabIndex        =   0
      Top             =   2280
      Width           =   1215
   End
   Begin VB.Label LblFirmwareVers 
      Caption         =   "Firmware Version"
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
      Left            =   240
      TabIndex        =   7
      ToolTipText     =   "Typ - Bezeichnung des Ladegeräts"
      Top             =   1800
      Width           =   1692
   End
   Begin VB.Label LblProdDat 
      Caption         =   "Produktions-Datum"
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
      Left            =   240
      TabIndex        =   5
      ToolTipText     =   "Typ - Bezeichnung des Ladegeräts"
      Top             =   1320
      Width           =   1812
   End
   Begin VB.Label LblSerNr 
      Caption         =   "Serie-Nummer"
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
      Left            =   240
      TabIndex        =   3
      ToolTipText     =   "Typ - Bezeichnung des Ladegeräts"
      Top             =   840
      Width           =   1572
   End
   Begin VB.Label LblGerTyp 
      Caption         =   "Geräte-Typ"
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
      Left            =   240
      TabIndex        =   1
      ToolTipText     =   "Typ - Bezeichnung des Ladegeräts"
      Top             =   360
      Width           =   1332
   End
End
Attribute VB_Name = "GeraeteInfo"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Name                            GeraeteInfo                                                           '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Beschreibung                    Das Info-Fensterchen der NLG - SW, zeigt die "Read-Only" Eigenschaften..'
'                                       des NLG in einer separaten, modalen Dialogbox an                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Was       / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / März '02  / Beat G.     / vorerst nur NLG-Typ                                                     '
' V2.0      / April '03 / Beat G.     / mehrsprachige Version (Deutsch / Englisch)                              '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
'

Option Explicit




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Info-Fensterchen laden, Titel und Texte anzeigen                                                              '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


Private Sub Form_Load()

GeraeteInfo.Caption = LoadResString(STR_ID_CHARGER_INFO_TITLE + g_lLocaleOffset)
GeraeteInfo.LblGerTyp = LoadResString(STR_ID_CHG_INFO_TYPE + g_lLocaleOffset)
GeraeteInfo.LblSerNr = LoadResString(STR_ID_CHG_INFO_SERIAL + g_lLocaleOffset)
GeraeteInfo.LblProdDat = LoadResString(STR_ID_CHG_INFO_PROD_DATE + g_lLocaleOffset)
GeraeteInfo.LblFirmwareVers = LoadResString(STR_ID_CHG_INFO_FW_VERSION + g_lLocaleOffset)


End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' OK - Button geklickt, also Geräte - Info - Dialog schliessen                                                  '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub OKButton_Click()
  Unload Me
End Sub
