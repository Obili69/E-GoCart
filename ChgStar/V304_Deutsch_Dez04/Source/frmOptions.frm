VERSION 5.00
Object = "{D940E4E4-6079-11CE-88CB-0020AF6845F6}#1.5#0"; "cwui.ocx"
Begin VB.Form FrmOptions 
   BorderStyle     =   1  'Fest Einfach
   Caption         =   "Options"
   ClientHeight    =   4056
   ClientLeft      =   36
   ClientTop       =   312
   ClientWidth     =   5472
   ControlBox      =   0   'False
   Icon            =   "frmOptions.frx":0000
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   4056
   ScaleWidth      =   5472
   Begin VB.CheckBox OptCPVentReq 
      Caption         =   "Ventilation Request"
      Height          =   192
      Left            =   240
      TabIndex        =   9
      Top             =   3600
      Width           =   1932
   End
   Begin VB.Frame OptCPFrame 
      Caption         =   "Control Pilot"
      Height          =   525
      Left            =   120
      TabIndex        =   23
      Top             =   3360
      Width           =   2172
   End
   Begin VB.CommandButton OptCancel 
      Cancel          =   -1  'True
      Caption         =   "Cancel"
      Default         =   -1  'True
      Height          =   372
      Left            =   3960
      TabIndex        =   11
      Top             =   3480
      Width           =   1212
   End
   Begin VB.CommandButton OptDefault 
      Caption         =   "Set CAN Defaults"
      Height          =   372
      Left            =   360
      TabIndex        =   2
      Top             =   2160
      Width           =   1452
   End
   Begin VB.Frame OptCANFrame 
      Caption         =   "CAN Parameter"
      Height          =   3132
      Left            =   120
      TabIndex        =   22
      Top             =   120
      Width           =   5250
      Begin VB.TextBox txtRxTxID 
         Height          =   288
         Index           =   5
         Left            =   2400
         Locked          =   -1  'True
         TabIndex        =   20
         TabStop         =   0   'False
         Text            =   "NLG5_ERR"
         Top             =   2640
         Width           =   1452
      End
      Begin VB.TextBox txtRxTxID 
         Height          =   288
         Index           =   4
         Left            =   2400
         Locked          =   -1  'True
         TabIndex        =   19
         TabStop         =   0   'False
         Text            =   "NLG5_TEMP"
         Top             =   2280
         Width           =   1452
      End
      Begin VB.TextBox txtRxTxID 
         Height          =   288
         Index           =   3
         Left            =   2400
         Locked          =   -1  'True
         TabIndex        =   18
         TabStop         =   0   'False
         Text            =   "NLG5_ACT_II"
         Top             =   1920
         Width           =   1452
      End
      Begin VB.TextBox txtRxTxID 
         Height          =   288
         Index           =   2
         Left            =   2400
         Locked          =   -1  'True
         TabIndex        =   17
         TabStop         =   0   'False
         Text            =   "NLG5_ACT_I"
         Top             =   1560
         Width           =   1452
      End
      Begin VB.TextBox txtRxTxID 
         Height          =   288
         Index           =   1
         Left            =   2400
         Locked          =   -1  'True
         TabIndex        =   16
         TabStop         =   0   'False
         Text            =   "NLG5_ST"
         Top             =   1200
         Width           =   1452
      End
      Begin VB.TextBox txtRxTxID 
         CausesValidation=   0   'False
         Height          =   288
         Index           =   0
         Left            =   2400
         Locked          =   -1  'True
         TabIndex        =   0
         TabStop         =   0   'False
         Text            =   "NLG5_CTL"
         Top             =   480
         Width           =   1452
      End
      Begin CWUIControlsLib.CWNumEdit NumEdRxTxID 
         Height          =   288
         Index           =   0
         Left            =   3960
         TabIndex        =   3
         Top             =   480
         Width           =   1092
         _Version        =   196609
         _ExtentX        =   1926
         _ExtentY        =   508
         _StockProps     =   4
         BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
            Name            =   "MS Sans Serif"
            Size            =   7.8
            Charset         =   0
            Weight          =   400
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Reset_0         =   0   'False
         CompatibleVers_0=   196609
         NumEdit_0       =   1
         ClassName_1     =   "CCWNumEdit"
         opts_1          =   458814
         BorderStyle_1   =   1
         format_1        =   2
         ClassName_2     =   "CCWFormat"
         Format_2        =   """0x""3X"
         scale_1         =   3
         ClassName_3     =   "CCWScale"
         dMax_3          =   10
         discInterval_3  =   1
         ValueVarType_1  =   5
         IncValueVarType_1=   5
         IncValue_Val_1  =   1
         AccelIncVarType_1=   5
         AccelInc_Val_1  =   5
         RangeMinVarType_1=   5
         RangeMaxVarType_1=   5
         RangeMax_Val_1  =   2047
         Bindings_1      =   4
         ClassName_4     =   "CCWBindingHolderArray"
         Editor_4        =   5
         ClassName_5     =   "CCWBindingHolderArrayEditor"
         Owner_5         =   1
      End
      Begin VB.ComboBox OptCANBaud 
         Height          =   288
         ItemData        =   "frmOptions.frx":0CCA
         Left            =   240
         List            =   "frmOptions.frx":0CDA
         TabIndex        =   1
         Text            =   "Combo1"
         Top             =   1200
         Width           =   1452
      End
      Begin CWUIControlsLib.CWNumEdit NumEdRxTxID 
         Height          =   288
         Index           =   1
         Left            =   3960
         TabIndex        =   4
         Top             =   1200
         Width           =   1092
         _Version        =   196609
         _ExtentX        =   1926
         _ExtentY        =   508
         _StockProps     =   4
         BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
            Name            =   "MS Sans Serif"
            Size            =   7.8
            Charset         =   0
            Weight          =   400
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Reset_0         =   0   'False
         CompatibleVers_0=   196609
         NumEdit_0       =   1
         ClassName_1     =   "CCWNumEdit"
         opts_1          =   458814
         BorderStyle_1   =   1
         format_1        =   2
         ClassName_2     =   "CCWFormat"
         Format_2        =   """0x""3X"
         scale_1         =   3
         ClassName_3     =   "CCWScale"
         dMax_3          =   10
         discInterval_3  =   1
         ValueVarType_1  =   5
         IncValueVarType_1=   5
         IncValue_Val_1  =   1
         AccelIncVarType_1=   5
         AccelInc_Val_1  =   5
         RangeMinVarType_1=   5
         RangeMaxVarType_1=   5
         RangeMax_Val_1  =   2047
         Bindings_1      =   4
         ClassName_4     =   "CCWBindingHolderArray"
         Editor_4        =   5
         ClassName_5     =   "CCWBindingHolderArrayEditor"
         Owner_5         =   1
      End
      Begin CWUIControlsLib.CWNumEdit NumEdRxTxID 
         Height          =   288
         Index           =   2
         Left            =   3960
         TabIndex        =   5
         Top             =   1560
         Width           =   1092
         _Version        =   196609
         _ExtentX        =   1926
         _ExtentY        =   508
         _StockProps     =   4
         BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
            Name            =   "MS Sans Serif"
            Size            =   7.8
            Charset         =   0
            Weight          =   400
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Reset_0         =   0   'False
         CompatibleVers_0=   196609
         NumEdit_0       =   1
         ClassName_1     =   "CCWNumEdit"
         opts_1          =   458814
         BorderStyle_1   =   1
         format_1        =   2
         ClassName_2     =   "CCWFormat"
         Format_2        =   """0x""3X"
         scale_1         =   3
         ClassName_3     =   "CCWScale"
         dMax_3          =   10
         discInterval_3  =   1
         ValueVarType_1  =   5
         IncValueVarType_1=   5
         IncValue_Val_1  =   1
         AccelIncVarType_1=   5
         AccelInc_Val_1  =   5
         RangeMinVarType_1=   5
         RangeMaxVarType_1=   5
         RangeMax_Val_1  =   2047
         Bindings_1      =   4
         ClassName_4     =   "CCWBindingHolderArray"
         Editor_4        =   5
         ClassName_5     =   "CCWBindingHolderArrayEditor"
         Owner_5         =   1
      End
      Begin CWUIControlsLib.CWNumEdit NumEdRxTxID 
         Height          =   288
         Index           =   3
         Left            =   3960
         TabIndex        =   6
         Top             =   1920
         Width           =   1092
         _Version        =   196609
         _ExtentX        =   1926
         _ExtentY        =   508
         _StockProps     =   4
         BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
            Name            =   "MS Sans Serif"
            Size            =   7.8
            Charset         =   0
            Weight          =   400
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Reset_0         =   0   'False
         CompatibleVers_0=   196609
         NumEdit_0       =   1
         ClassName_1     =   "CCWNumEdit"
         opts_1          =   458814
         BorderStyle_1   =   1
         format_1        =   2
         ClassName_2     =   "CCWFormat"
         Format_2        =   """0x""3X"
         scale_1         =   3
         ClassName_3     =   "CCWScale"
         dMax_3          =   10
         discInterval_3  =   1
         ValueVarType_1  =   5
         IncValueVarType_1=   5
         IncValue_Val_1  =   1
         AccelIncVarType_1=   5
         AccelInc_Val_1  =   5
         RangeMinVarType_1=   5
         RangeMaxVarType_1=   5
         RangeMax_Val_1  =   2047
         Bindings_1      =   4
         ClassName_4     =   "CCWBindingHolderArray"
         Editor_4        =   5
         ClassName_5     =   "CCWBindingHolderArrayEditor"
         Owner_5         =   1
      End
      Begin CWUIControlsLib.CWNumEdit NumEdRxTxID 
         Height          =   288
         Index           =   4
         Left            =   3960
         TabIndex        =   7
         Top             =   2280
         Width           =   1092
         _Version        =   196609
         _ExtentX        =   1926
         _ExtentY        =   508
         _StockProps     =   4
         BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
            Name            =   "MS Sans Serif"
            Size            =   7.8
            Charset         =   0
            Weight          =   400
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Reset_0         =   0   'False
         CompatibleVers_0=   196609
         NumEdit_0       =   1
         ClassName_1     =   "CCWNumEdit"
         opts_1          =   458814
         BorderStyle_1   =   1
         format_1        =   2
         ClassName_2     =   "CCWFormat"
         Format_2        =   """0x""3X"
         scale_1         =   3
         ClassName_3     =   "CCWScale"
         dMax_3          =   10
         discInterval_3  =   1
         ValueVarType_1  =   5
         IncValueVarType_1=   5
         IncValue_Val_1  =   1
         AccelIncVarType_1=   5
         AccelInc_Val_1  =   5
         RangeMinVarType_1=   5
         RangeMaxVarType_1=   5
         RangeMax_Val_1  =   2047
         Bindings_1      =   4
         ClassName_4     =   "CCWBindingHolderArray"
         Editor_4        =   5
         ClassName_5     =   "CCWBindingHolderArrayEditor"
         Owner_5         =   1
      End
      Begin CWUIControlsLib.CWNumEdit NumEdRxTxID 
         Height          =   288
         Index           =   5
         Left            =   3960
         TabIndex        =   8
         Top             =   2640
         Width           =   1092
         _Version        =   196609
         _ExtentX        =   1926
         _ExtentY        =   508
         _StockProps     =   4
         BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
            Name            =   "MS Sans Serif"
            Size            =   7.8
            Charset         =   0
            Weight          =   400
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Reset_0         =   0   'False
         CompatibleVers_0=   196609
         NumEdit_0       =   1
         ClassName_1     =   "CCWNumEdit"
         opts_1          =   458814
         BorderStyle_1   =   1
         format_1        =   2
         ClassName_2     =   "CCWFormat"
         Format_2        =   """0x""3X"
         scale_1         =   3
         ClassName_3     =   "CCWScale"
         dMax_3          =   10
         discInterval_3  =   1
         ValueVarType_1  =   5
         IncValueVarType_1=   5
         IncValue_Val_1  =   1
         AccelIncVarType_1=   5
         AccelInc_Val_1  =   5
         RangeMinVarType_1=   5
         RangeMaxVarType_1=   5
         RangeMax_Val_1  =   2047
         Bindings_1      =   4
         ClassName_4     =   "CCWBindingHolderArray"
         Editor_4        =   5
         ClassName_5     =   "CCWBindingHolderArrayEditor"
         Owner_5         =   1
      End
      Begin VB.Label lblTxId 
         Caption         =   "Identifier (hex)"
         Height          =   252
         Left            =   3960
         TabIndex        =   12
         Top             =   960
         Width           =   1200
      End
      Begin VB.Label lblRxId 
         Caption         =   "Identifier (hex)"
         Height          =   252
         Left            =   3960
         TabIndex        =   15
         Top             =   240
         Width           =   1200
      End
      Begin VB.Label lblTxMsg 
         Caption         =   "Transmit Messages"
         Height          =   252
         Left            =   2400
         TabIndex        =   13
         Top             =   960
         Width           =   1500
      End
      Begin VB.Label lblRxMsg 
         Caption         =   "Received Message"
         Height          =   252
         Left            =   2400
         TabIndex        =   14
         Top             =   240
         Width           =   1500
      End
      Begin VB.Label lblBaud 
         Caption         =   "CAN Baudrate"
         Height          =   252
         Left            =   240
         TabIndex        =   21
         Top             =   960
         Width           =   1452
      End
   End
   Begin VB.CommandButton OptOk 
      Caption         =   "OK"
      Height          =   372
      Left            =   2520
      TabIndex        =   10
      Top             =   3480
      Width           =   1212
   End
End
Attribute VB_Name = "FrmOptions"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Name                            FrmOptions                                                              '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Beschreibung                    Das Fensterchen zeigt die Optionen an, die gewählt werden können.       '
'                                       - CAN Baudrate                                                          '
'                                       - CAN Identifier für Rx und Tx Messages                                 '
'                                       - Control Pilot: Ventilation Request ON/OFF                             '
'                                                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Was       / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / Juli '03  / Beat G.     / Für Version 2.0                                                         '
' geändert  / Sept '04  / Beat G.     / Für Version 3.0, bg022, Parametrierung von CAN Baudrate und ID's        '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
'                                     /                                                                         '
' Form_Load                           / Dialogbox laden für CAN Parameter oder Auto (nur CP vent. req.)         '
' OptDefault_Click                    / Default CAN Parameter laden                                             '
' OptOK_Click                         / Einstellungen in die Datenkapsel übernehmen, CAN ID's prüfen            '
' ***_Click, ***_Changed              / Eingaben von GUI-Controls behandeln, rufen i.d.R. Dienstfunktionen auf  '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
'
'
'

Option Explicit

'
'-------------------- lokale Konstanten, für CAN Parametrierung -----------------

' Index auf dem GUI der RX / TX Controls
Const CTRL_IDX_CAN_RXTX_MSG_ID_CTL      As Integer = 0
Const CTRL_IDX_CAN_RXTX_MSG_ID_ST       As Integer = 1
Const CTRL_IDX_CAN_RXTX_MSG_ID_ACT_I    As Integer = 2
Const CTRL_IDX_CAN_RXTX_MSG_ID_ACT_II   As Integer = 3
Const CTRL_IDX_CAN_RXTX_MSG_ID_TEMP     As Integer = 4
Const CTRL_IDX_CAN_RXTX_MSG_ID_ERR      As Integer = 5




' -------------- modulglobale Variablen ------------------------

Dim bAnyValueChanged      As Boolean  ' irgendein Wert wurde geändert




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fensterchen laden, Titel etc. anzeigen.                                                                       '
' Info "Vent.Request" on/off, CAN ID's, CAN Baudrate etc. aus Datenkapsel laden und anzeigen                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_Load()
  
  Dim iLoop As Integer
  Dim iAux As Integer
  
  ' Optik und Sprache/Bezeichnungen laden
  Me.Caption = LoadResString(STR_ID_OPTIONS_TITLE + g_lLocaleOffset)                          ' Fenster-Titel laden
  FrmOptions.OptCANFrame.Caption = LoadResString(STR_ID_OPT_CAN_FRAME + g_lLocaleOffset)      ' CAN Rahmen
  FrmOptions.lblBaud.Caption = LoadResString(STR_ID_OPT_LBL_BAUD + g_lLocaleOffset)           ' Baud Rate
  FrmOptions.OptDefault.Caption = LoadResString(STR_ID_OPT_DEFAULT + g_lLocaleOffset)         ' CAN Defaults
  FrmOptions.OptCPFrame.Caption = LoadResString(STR_ID_OPTIONS_CP_FRAME + g_lLocaleOffset)    ' Control Pilot Rahmen
  FrmOptions.OptCPVentReq.Caption = LoadResString(STR_ID_OPT_VENT_REQUEST + g_lLocaleOffset)  ' CP ventilation request
  FrmOptions.lblRxMsg.Caption = LoadResString(STR_ID_OPT_LBL_RX_MSG + g_lLocaleOffset)        ' Receive Messages
  FrmOptions.lblTxMsg.Caption = LoadResString(STR_ID_OPT_LBL_TX_MSG + g_lLocaleOffset)        ' Transmit Messages
  FrmOptions.lblRxId.Caption = LoadResString(STR_ID_OPT_LBL_RXTX_ID + g_lLocaleOffset)        ' Rx ID (hex)
  FrmOptions.lblTxId.Caption = LoadResString(STR_ID_OPT_LBL_RXTX_ID + g_lLocaleOffset)        ' Tx ID (hex)
  FrmOptions.OptCancel.Caption = LoadResString(STR_ID_TXT_CANCEL + g_lLocaleOffset)           ' Cancel Button
  
  ' CAN Baud Rate Listbox mit Inhalten füllen
  OptCANBaud.ItemData(INDEX_CAN_BAUD_125K) = CAN_BAUDRATE_125k
  OptCANBaud.ItemData(INDEX_CAN_BAUD_250K) = CAN_BAUDRATE_250k
  OptCANBaud.ItemData(INDEX_CAN_BAUD_500K) = CAN_BAUDRATE_500k
  OptCANBaud.ItemData(INDEX_CAN_BAUD_1M) = CAN_BAUDRATE_1M

  ' CAN Default Werte laden, für alle Fälle
  Call OptDefault_Click                                         ' Msg ID's
  OptCANBaud.ListIndex = DEFAULT_INDEX_CAN_BAUD                 ' Baudrate


  ' --- Daten aus Kapsel laden und in GUI übernehmen
  OptCPVentReq.Value = FnLimit(GetDataFromCapsule(FnHiByteOfLong(VENTI_REQ_BIT_CP), FnLoByteOfLong(VENTI_REQ_BIT_CP)), BFALSE, BTRUE) ' Vent.Request Ein/Aus von Datenkapsel lesen
  NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_CTL).Value = FnLimit(GetDataFromCapsule(FnHiByteOfLong(CAN_RX_ID_CTL___), FnLoByteOfLong(CAN_RX_ID_CTL___)), MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID) ' Vent.Request Ein/Aus von Datenkapsel lesen
  NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ST).Value = FnLimit(GetDataFromCapsule(FnHiByteOfLong(CAN_TX_ID_ST____), FnLoByteOfLong(CAN_TX_ID_ST____)), MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID) ' Vent.Request Ein/Aus von Datenkapsel lesen
  NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ACT_I).Value = FnLimit(GetDataFromCapsule(FnHiByteOfLong(CAN_TX_ID_ACT_I_), FnLoByteOfLong(CAN_TX_ID_ACT_I_)), MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID) ' Vent.Request Ein/Aus von Datenkapsel lesen
  NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ACT_II).Value = FnLimit(GetDataFromCapsule(FnHiByteOfLong(CAN_TX_ID_ACT_II), FnLoByteOfLong(CAN_TX_ID_ACT_II)), MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID) ' Vent.Request Ein/Aus von Datenkapsel lesen
  NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_TEMP).Value = FnLimit(GetDataFromCapsule(FnHiByteOfLong(CAN_TX_ID_TEMP__), FnLoByteOfLong(CAN_TX_ID_TEMP__)), MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID) ' Vent.Request Ein/Aus von Datenkapsel lesen
  NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ERR).Value = FnLimit(GetDataFromCapsule(FnHiByteOfLong(CAN_TX_ID_ERR___), FnLoByteOfLong(CAN_TX_ID_ERR___)), MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID) ' Vent.Request Ein/Aus von Datenkapsel lesen
  
  ' Binären Wert für CAN Baudrate aus Kapsel lesen und entsprechenden Index setzen
  iAux = FnLimit(GetDataFromCapsule(FnHiByteOfLong(CAN_BAUD_RATE___), FnLoByteOfLong(CAN_BAUD_RATE___)), CAN_BAUDRATE_1M, CAN_BAUDRATE_500k) ' Baudrate lesen ACHTUNG: die Werte sind NICHT aufsteigend geordnet (1M..125k)
  For iLoop = INDEX_CAN_BAUD_1M To INDEX_CAN_BAUD_125K
    If (OptCANBaud.ItemData(iLoop) = iAux) Then                 ' falls gültiger Wert aus Kapsel, Default überschreiben..
      OptCANBaud.ListIndex = iLoop                              '..mit passendem Index
    End If
  Next iLoop
  
  ' --- je nach Betriebsart Controls anzeigen oder verbergen
  If (GetDataFromCapsule(0, BETR_MASTR_BOOST) = C_CAN) Then     ' bei CAN Betrieb hat das "Ventilation Request" Bit..
    FrmOptions.OptCPVentReq.Enabled = False                     '..keinen Einfluss (wird von CAN-Matrix gesetzt)
  Else                                                          ' Auto oder Booster
    FrmOptions.OptCPVentReq.Enabled = True                      ' ventilation request Bit aktiv
    FrmOptions.OptCANBaud.Enabled = False                       ' hingegen sind die CAN Controls inaktiv
    FrmOptions.OptDefault.Enabled = False
    For iLoop = CTRL_IDX_CAN_RXTX_MSG_ID_CTL To CTRL_IDX_CAN_RXTX_MSG_ID_ERR
      FrmOptions.NumEdRxTxID(iLoop).Enabled = False
      FrmOptions.txtRxTxID(iLoop).Enabled = False
    Next iLoop
  
  End If
  
  bAnyValueChanged = False                                      ' initialisieren, nix geändert bis jetzt
    
End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Irgendein Wert der Rx oder Tx ID geändert, also neuen Wert übernehmen                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Private Sub NumEdRxTxID_ValueChanged(Index As Integer, Value As Variant, PreviousValue As Variant, ByVal OutOfRange As Boolean)

  bAnyValueChanged = True                                       ' Wert geändert
  
End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' CAN Baudrate geändert, also neuen Wert übernehmen                                                             '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub OptCANBaud_Click()

  bAnyValueChanged = True                                       ' Wert geändert

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' "Vent.Request" on/off getoggelt, neuen Wert übernehmen                                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub OptCPVentReq_Click()
  
  bAnyValueChanged = True                                       ' Wert geändert

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Default CAN Einstellungen laden                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub OptDefault_Click()

  ' Fülle CAN Baudrate und ID's mit Default Werten (nur Oberfläche)
  OptCANBaud.ListIndex = DEFAULT_INDEX_CAN_BAUD
  FrmOptions.NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_CTL).Value = DEFAULT_RX_CAN_ID_NLG5_CTL
  FrmOptions.NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ST).Value = DEFAULT_TX_CAN_ID_NLG5_ST
  FrmOptions.NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ACT_I).Value = DEFAULT_TX_CAN_ID_NLG5_ACT_I
  FrmOptions.NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ACT_II).Value = DEFAULT_TX_CAN_ID_NLG5_ACT_II
  FrmOptions.NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_TEMP).Value = DEFAULT_TX_CAN_ID_NLG5_TEMP
  FrmOptions.NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ERR).Value = DEFAULT_TX_CAN_ID_NLG5_ERR

  bAnyValueChanged = True                                       ' Wert geändert

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fensterchen verlassen, Daten speichern falls geändert                                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub OptOK_Click()
  
  Dim bInputError As Boolean                                    ' ungültige Eingabe, gleiche CAN ID's
  Dim iOuterLoop As Integer                                     ' äussere Schleife
  Dim iInnerLoop As Integer                                     ' innere Schleife
  
  
  bInputError = False                                           ' checken, ob alle ID's voneinander verschieden sind
  
  For iOuterLoop = (CTRL_IDX_CAN_RXTX_MSG_ID_CTL) To (CTRL_IDX_CAN_RXTX_MSG_ID_ERR - 1)
    For iInnerLoop = (iOuterLoop + 1) To CTRL_IDX_CAN_RXTX_MSG_ID_ERR
      If (FrmOptions.NumEdRxTxID(iOuterLoop).Value = FrmOptions.NumEdRxTxID(iInnerLoop).Value) Then
        bInputError = True                                      ' diese zwei sind gleich
      End If
    Next iInnerLoop
  Next iOuterLoop
  
  If (bInputError = True) Then          ' 2 oder mehr gleiche ID's, zurück zu den Controls, nix speichern oder so
    MsgBox (LoadResString(STR_ID_USE_DIFFERENT_CAN_ID + g_lLocaleOffset))
  Else
    If (bAnyValueChanged = True) Then                             ' Wenn was geändert, dann alles in die Datenkapsel
      Call PutDataToCapsule(FnHiByteOfLong(VENTI_REQ_BIT_CP), FnLoByteOfLong(VENTI_REQ_BIT_CP), FnLimit(OptCPVentReq.Value, BFALSE, BTRUE))
      Call PutDataToCapsule(FnHiByteOfLong(CAN_BAUD_RATE___), FnLoByteOfLong(CAN_BAUD_RATE___), FnLimit(OptCANBaud.ItemData(OptCANBaud.ListIndex), CAN_BAUDRATE_1M, CAN_BAUDRATE_500k))    ' Achtung: Wertebereich "unlogisch" auf Register abgebildet
      Call PutDataToCapsule(FnHiByteOfLong(CAN_RX_ID_CTL___), FnLoByteOfLong(CAN_RX_ID_CTL___), FnLimit(NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_CTL).Value, MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID))
      Call PutDataToCapsule(FnHiByteOfLong(CAN_TX_ID_ST____), FnLoByteOfLong(CAN_TX_ID_ST____), FnLimit(NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ST).Value, MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID))
      Call PutDataToCapsule(FnHiByteOfLong(CAN_TX_ID_ACT_I_), FnLoByteOfLong(CAN_TX_ID_ACT_I_), FnLimit(NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ACT_I).Value, MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID))
      Call PutDataToCapsule(FnHiByteOfLong(CAN_TX_ID_ACT_II), FnLoByteOfLong(CAN_TX_ID_ACT_II), FnLimit(NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ACT_II).Value, MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID))
      Call PutDataToCapsule(FnHiByteOfLong(CAN_TX_ID_TEMP__), FnLoByteOfLong(CAN_TX_ID_TEMP__), FnLimit(NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_TEMP).Value, MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID))
      Call PutDataToCapsule(FnHiByteOfLong(CAN_TX_ID_ERR___), FnLoByteOfLong(CAN_TX_ID_ERR___), FnLimit(NumEdRxTxID(CTRL_IDX_CAN_RXTX_MSG_ID_ERR).Value, MIN_RX_TX_CAN_ID, MAX_RX_TX_CAN_ID))
      g_bProfileChanged = True                                    ' Merker "wolle Du speichern?" setzen
    End If
    Unload Me                                                     ' Fenster verlassen
  End If

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fensterchen verlassen, nix speichern                                                                          '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Private Sub OptCancel_Click()
  
  ' Alles Zeugs verwerfen
  Unload Me             ' Fenster verlassen

End Sub

