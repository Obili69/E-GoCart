VERSION 5.00
Object = "{648A5603-2C6E-101B-82B6-000000000014}#1.1#0"; "MSCOMM32.OCX"
Object = "{F9043C88-F6F2-101A-A3C9-08002B2F49FB}#1.2#0"; "COMDLG32.OCX"
Object = "{D940E4E4-6079-11CE-88CB-0020AF6845F6}#1.5#0"; "cwui.ocx"
Begin VB.Form kennlinie 
   BackColor       =   &H00E0E0E0&
   Caption         =   "ChargeStar NLG 5"
   ClientHeight    =   8232
   ClientLeft      =   168
   ClientTop       =   516
   ClientWidth     =   13548
   FillColor       =   &H00FF0000&
   BeginProperty Font 
      Name            =   "Arial"
      Size            =   9
      Charset         =   0
      Weight          =   700
      Underline       =   0   'False
      Italic          =   0   'False
      Strikethrough   =   0   'False
   EndProperty
   ForeColor       =   &H80000011&
   HelpContextID   =   10
   Icon            =   "Kennlinie.frx":0000
   LinkTopic       =   "Form3"
   ScaleHeight     =   8245.005
   ScaleMode       =   0  'Benutzerdefiniert
   ScaleWidth      =   16238.13
   StartUpPosition =   1  'Fenstermitte
   Begin VB.TextBox NLGTyp 
      Appearance      =   0  '2D
      BackColor       =   &H00E0E0E0&
      BorderStyle     =   0  'Kein
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      HelpContextID   =   100
      Left            =   4800
      Locked          =   -1  'True
      TabIndex        =   102
      TabStop         =   0   'False
      Text            =   " - - -"
      Top             =   120
      Width           =   1572
   End
   Begin VB.Timer BlinkTimer 
      Interval        =   1
      Left            =   13080
      Top             =   6360
   End
   Begin VB.TextBox TextMaxIBoost 
      BackColor       =   &H00C0C0C0&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   170
      Left            =   6120
      Locked          =   -1  'True
      TabIndex        =   99
      TabStop         =   0   'False
      Text            =   "Ext. Netzstrom für volle Booster-Leistung  [A]"
      Top             =   1440
      Width           =   3252
   End
   Begin VB.TextBox TextMinIBoost 
      BackColor       =   &H00C0C0C0&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   170
      Left            =   6120
      Locked          =   -1  'True
      TabIndex        =   98
      TabStop         =   0   'False
      Text            =   "Ext. Netzstrom für  Booster Aus   [A]"
      Top             =   1080
      Width           =   3252
   End
   Begin VB.ComboBox ComboBetriebsart 
      BackColor       =   &H00FFFFFF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   288
      HelpContextID   =   170
      ItemData        =   "Kennlinie.frx":0CCA
      Left            =   9480
      List            =   "Kennlinie.frx":0CD7
      Style           =   2  'Dropdown-Liste
      TabIndex        =   16
      Top             =   720
      Width           =   1092
   End
   Begin VB.CheckBox CheckTempSens 
      Caption         =   "#3"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      HelpContextID   =   100
      Index           =   0
      Left            =   5040
      TabIndex        =   11
      Top             =   1800
      Width           =   555
   End
   Begin VB.CheckBox CheckTempSens 
      Caption         =   "#2"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      HelpContextID   =   100
      Index           =   1
      Left            =   4320
      TabIndex        =   10
      Top             =   1800
      Width           =   555
   End
   Begin VB.CheckBox CheckTempSens 
      Caption         =   "#1"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      HelpContextID   =   100
      Index           =   2
      Left            =   3600
      TabIndex        =   9
      Top             =   1800
      Width           =   555
   End
   Begin VB.TextBox TextBetriebsart 
      BackColor       =   &H00C0C0C0&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   170
      Left            =   6120
      Locked          =   -1  'True
      TabIndex        =   97
      TabStop         =   0   'False
      Text            =   "NLG Betriebsart"
      Top             =   720
      Width           =   3252
   End
   Begin VB.TextBox TextTempSensoren 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      Locked          =   -1  'True
      TabIndex        =   96
      TabStop         =   0   'False
      Text            =   "Aktive Temperatursensoren"
      Top             =   1800
      Width           =   3252
   End
   Begin VB.CommandButton AbschnVor 
      Caption         =   "Weiter >>"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   300
      HelpContextID   =   300
      Left            =   6120
      TabIndex        =   26
      Top             =   4080
      Width           =   1092
   End
   Begin VB.CommandButton AbschnZurueck 
      Caption         =   "<< Zurück   "
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   300
      HelpContextID   =   300
      Left            =   3360
      TabIndex        =   24
      Top             =   4080
      Width           =   1092
   End
   Begin MSComDlg.CommonDialog datei_fenster1 
      Left            =   13080
      Top             =   6840
      _ExtentX        =   847
      _ExtentY        =   847
      _Version        =   393216
      CancelError     =   -1  'True
   End
   Begin VB.TextBox TextUnenn 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      Locked          =   -1  'True
      TabIndex        =   91
      TabStop         =   0   'False
      Text            =   "Nennspannung der Batterie   [V]"
      Top             =   720
      Width           =   4245
   End
   Begin VB.TextBox TextTempKoeff 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      Locked          =   -1  'True
      TabIndex        =   90
      TabStop         =   0   'False
      Text            =   "Temperatur-Koeffizient der Gasungsspannung   [%/°C]"
      Top             =   1080
      Width           =   4245
   End
   Begin VB.TextBox TextRefTemp 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      Locked          =   -1  'True
      TabIndex        =   89
      TabStop         =   0   'False
      Text            =   "Bezugstemperatur für Spannungsangaben   [°C]"
      Top             =   1440
      Width           =   4245
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0C000&
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   10
      Index           =   30
      Left            =   13080
      Locked          =   -1  'True
      TabIndex        =   88
      TabStop         =   0   'False
      Text            =   "maximale Netzleistung   [Watt]"
      Top             =   5640
      Width           =   4245
   End
   Begin VB.TextBox TextMaxINetz 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      Locked          =   -1  'True
      TabIndex        =   87
      TabStop         =   0   'False
      Text            =   "Maximaler Netzstrom   [A]"
      Top             =   2280
      Width           =   4245
   End
   Begin VB.TextBox TextBatTempDerat 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      Locked          =   -1  'True
      TabIndex        =   86
      TabStop         =   0   'False
      Text            =   "Batterie-Temperatur-Derating   [A/°C]"
      Top             =   3120
      Width           =   4245
   End
   Begin VB.TextBox TextBezTempDerat 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      Locked          =   -1  'True
      TabIndex        =   85
      TabStop         =   0   'False
      Text            =   "Bezugstemperatur für Derating auf Null [°C]"
      Top             =   3480
      Width           =   4245
   End
   Begin VB.TextBox TextTempUeb 
      BackColor       =   &H0080C0FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   200
      Left            =   6120
      Locked          =   -1  'True
      TabIndex        =   84
      TabStop         =   0   'False
      Text            =   "Batterie-Temperatur über   [°C]"
      Top             =   2400
      Width           =   3252
   End
   Begin VB.TextBox TextSpgUeb 
      BackColor       =   &H0080C0FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   200
      Left            =   6120
      Locked          =   -1  'True
      TabIndex        =   83
      TabStop         =   0   'False
      Text            =   "Batterie-Spannung über   [V]"
      Top             =   2760
      Width           =   3252
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H0080C0FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   200
      Index           =   37
      Left            =   13080
      Locked          =   -1  'True
      TabIndex        =   82
      TabStop         =   0   'False
      Text            =   "Bezogene Netz-Energie mehr als   [kWh]"
      Top             =   6000
      Width           =   3405
   End
   Begin VB.TextBox TextLadUeb 
      BackColor       =   &H0080C0FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   200
      Left            =   6120
      Locked          =   -1  'True
      TabIndex        =   81
      TabStop         =   0   'False
      Text            =   "Batterie-Ladung mehr als   [Ah]"
      Top             =   3120
      Width           =   3252
   End
   Begin VB.TextBox TextZeitUeb 
      BackColor       =   &H0080C0FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   200
      Left            =   6120
      Locked          =   -1  'True
      TabIndex        =   80
      TabStop         =   0   'False
      Text            =   "Ladezeit länger als   [h]"
      Top             =   3480
      Width           =   3252
   End
   Begin VB.ComboBox Reduktions_art 
      BackColor       =   &H00FFFFFF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   288
      HelpContextID   =   100
      ItemData        =   "Kennlinie.frx":0CEF
      Left            =   4560
      List            =   "Kennlinie.frx":0CF9
      Style           =   2  'Dropdown-Liste
      TabIndex        =   13
      Top             =   2760
      Width           =   1092
   End
   Begin VB.TextBox TextTempRedArt 
      BackColor       =   &H00C0C000&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   100
      Left            =   240
      TabIndex        =   79
      TabStop         =   0   'False
      Text            =   "Temperatur-Reduktions-Art"
      Top             =   2760
      Width           =   4245
   End
   Begin VB.ComboBox ComboWechselOption 
      BackColor       =   &H00FFFFFF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   288
      ItemData        =   "Kennlinie.frx":0D12
      Left            =   4320
      List            =   "Kennlinie.frx":0D2E
      TabIndex        =   76
      Text            =   "Combo2"
      Top             =   10800
      Width           =   1575
   End
   Begin VB.ListBox BedingTypVW 
      Appearance      =   0  '2D
      BackColor       =   &H00FFFFFF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   1752
      HelpContextID   =   350
      ItemData        =   "Kennlinie.frx":0D87
      Left            =   360
      List            =   "Kennlinie.frx":0D89
      TabIndex        =   37
      ToolTipText     =   "Doppelklick auf Bedingung fügt diese hinzu"
      Top             =   6240
      Width           =   4452
   End
   Begin VB.ListBox BedingTypRW 
      Appearance      =   0  '2D
      BackColor       =   &H00FFFFFF&
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   216
      Left            =   13680
      TabIndex        =   75
      ToolTipText     =   "Auswahl mit Doppelklick"
      Top             =   3480
      Width           =   1455
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   5
      Left            =   14160
      TabIndex        =   65
      Top             =   8160
      Visible         =   0   'False
      Width           =   855
      _Version        =   196609
      _ExtentX        =   1508
      _ExtentY        =   450
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWButton CWButton2 
      Height          =   375
      Index           =   4
      Left            =   15720
      TabIndex        =   53
      Top             =   7560
      Visible         =   0   'False
      Width           =   375
      _Version        =   196609
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.86
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "And"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "Or"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin VB.CheckBox Check1 
      Caption         =   "Quittiersignale 1 + 2"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   4
      Left            =   12840
      TabIndex        =   52
      Top             =   2520
      Width           =   1695
   End
   Begin VB.CheckBox Check1 
      Caption         =   "Quittiersignal 2"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   3
      Left            =   12840
      TabIndex        =   51
      Top             =   2280
      Width           =   1575
   End
   Begin VB.CheckBox Check1 
      Caption         =   "Quittiersignal 1"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   2
      Left            =   12840
      TabIndex        =   50
      Top             =   2040
      Width           =   1575
   End
   Begin VB.CheckBox Check1 
      Caption         =   "Aus / Einschalten"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   1
      Left            =   12840
      TabIndex        =   49
      Top             =   1800
      Width           =   1575
   End
   Begin CWUIControlsLib.CWNumEdit num1 
      Height          =   288
      HelpContextID   =   320
      Index           =   4
      Left            =   8280
      TabIndex        =   33
      Top             =   5160
      Visible         =   0   'False
      Width           =   852
      _Version        =   196609
      _ExtentX        =   1503
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
      opts_1          =   327742
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   50
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   50
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin VB.TextBox Text8 
      BackColor       =   &H80000013&
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   320
      Left            =   5640
      Locked          =   -1  'True
      TabIndex        =   47
      TabStop         =   0   'False
      Text            =   "Zählt für Überladung  [%]"
      Top             =   5160
      Visible         =   0   'False
      Width           =   2532
   End
   Begin VB.TextBox TextStatusI 
      Alignment       =   2  'Zentriert
      BackColor       =   &H80000013&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   330
      Left            =   9360
      Locked          =   -1  'True
      TabIndex        =   4
      TabStop         =   0   'False
      Text            =   "Status"
      Top             =   4800
      Width           =   975
   End
   Begin VB.TextBox TextStatusU 
      Alignment       =   2  'Zentriert
      BackColor       =   &H80000013&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   330
      Left            =   4080
      Locked          =   -1  'True
      TabIndex        =   3
      TabStop         =   0   'False
      Text            =   "Status"
      Top             =   4800
      Width           =   975
   End
   Begin VB.TextBox Text3 
      BackColor       =   &H80000013&
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   320
      Index           =   0
      Left            =   360
      Locked          =   -1  'True
      TabIndex        =   45
      TabStop         =   0   'False
      Text            =   "Zählt für Überladung  [%]"
      Top             =   5160
      Visible         =   0   'False
      Width           =   2532
   End
   Begin VB.ComboBox ComboAbschnitt 
      BackColor       =   &H00FFFFFF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   288
      HelpContextID   =   300
      ItemData        =   "Kennlinie.frx":0D8B
      Left            =   4560
      List            =   "Kennlinie.frx":0D8D
      Style           =   2  'Dropdown-Liste
      TabIndex        =   25
      Top             =   4080
      Width           =   1455
   End
   Begin CWUIControlsLib.CWButton LEDButton 
      Height          =   252
      HelpContextID   =   330
      Index           =   3
      Left            =   4080
      TabIndex        =   29
      Top             =   5160
      Width           =   252
      _Version        =   196609
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.9
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   6686
      C[0]_1          =   14737632
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   286
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   -2147483640
      C[1]_7          =   -2147483640
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      style_8         =   1076101120
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   -2147483640
      C[1]_9          =   -2147483640
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      C[0]_11         =   12632256
      C[1]_11         =   12632256
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   73
      F_12            =   12632256
      B_12            =   12632256
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   286
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   -2147483640
      C[1]_16         =   -2147483640
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   -2147483640
      C[1]_17         =   -2147483640
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      C[0]_18         =   49152
      C[1]_18         =   49152
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   74
      F_19            =   49152
      B_19            =   49152
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   4
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin CWUIControlsLib.CWNumEdit num1 
      Height          =   288
      HelpContextID   =   320
      Index           =   2
      Left            =   8280
      TabIndex        =   32
      Top             =   4800
      Width           =   852
      _Version        =   196609
      _ExtentX        =   1503
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
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   720
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit num1 
      Height          =   288
      HelpContextID   =   320
      Index           =   1
      Left            =   3000
      TabIndex        =   27
      Top             =   4800
      Width           =   852
      _Version        =   196609
      _ExtentX        =   1503
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
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   75
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin VB.TextBox TextMaxSpg 
      BackColor       =   &H80000013&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   320
      Left            =   5640
      Locked          =   -1  'True
      TabIndex        =   42
      TabStop         =   0   'False
      Text            =   "Maximale Ladespannung  [V]"
      Top             =   4800
      Width           =   2532
   End
   Begin VB.TextBox TextMaxStrom 
      BackColor       =   &H80000013&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   285
      HelpContextID   =   320
      Left            =   360
      Locked          =   -1  'True
      TabIndex        =   5
      TabStop         =   0   'False
      Text            =   "Maximaler Ladestrom  [A]"
      Top             =   4800
      Width           =   2532
   End
   Begin CWUIControlsLib.CWButton LEDButton 
      Height          =   252
      HelpContextID   =   330
      Index           =   2
      Left            =   4440
      TabIndex        =   30
      Top             =   5160
      Width           =   252
      _Version        =   196609
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.9
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   6686
      C[0]_1          =   14737632
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   286
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   7
      ClassName_7     =   "CCWBlinker"
      Interval_7      =   600
      Enable_7        =   -1  'True
      EventID_7       =   7
      Obj_7           =   5
      list[3]_4       =   8
      ClassName_8     =   "CCWDrawObj"
      opts_8          =   60
      C[0]_8          =   -2147483640
      C[1]_8          =   -2147483640
      Image_8         =   9
      ClassName_9     =   "CCWTextImage"
      style_9         =   1076101120
      font_9          =   0
      Animator_8      =   0
      Blinker_8       =   0
      list[2]_4       =   10
      ClassName_10    =   "CCWDrawObj"
      opts_10         =   62
      C[0]_10         =   -2147483640
      C[1]_10         =   -2147483640
      Image_10        =   11
      ClassName_11    =   "CCWTextImage"
      font_11         =   0
      Animator_10     =   0
      Blinker_10      =   0
      list[1]_4       =   12
      ClassName_12    =   "CCWDrawObj"
      opts_12         =   62
      C[0]_12         =   12632256
      C[1]_12         =   12632256
      Image_12        =   13
      ClassName_13    =   "CCWPictImage"
      opts_13         =   1280
      Rows_13         =   1
      Cols_13         =   1
      Pict_13         =   73
      F_13            =   12632256
      B_13            =   12632256
      ColorReplaceWith_13=   8421504
      ColorReplace_13 =   8421504
      Tolerance_13    =   2
      Animator_12     =   0
      Blinker_12      =   0
      varVarType_3    =   5
      Array[1]_2      =   14
      ClassName_14    =   "CCWEnumElt"
      opts_14         =   1
      Name_14         =   "On"
      frame_14        =   286
      DrawList_14     =   15
      ClassName_15    =   "CDrawList"
      count_15        =   4
      list[4]_15      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   -2147483640
      C[1]_16         =   -2147483640
      Image_16        =   6
      Animator_16     =   0
      Blinker_16      =   0
      list[3]_15      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   62
      C[0]_17         =   -2147483640
      C[1]_17         =   -2147483640
      Image_17        =   9
      Animator_17     =   0
      Blinker_17      =   0
      list[2]_15      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   60
      C[0]_18         =   -2147483640
      C[1]_18         =   -2147483640
      Image_18        =   11
      Animator_18     =   0
      Blinker_18      =   0
      list[1]_15      =   19
      ClassName_19    =   "CCWDrawObj"
      opts_19         =   62
      C[0]_19         =   65535
      C[1]_19         =   65535
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   65535
      B_20            =   65535
      ColorReplaceWith_20=   8421504
      ColorReplace_20 =   8421504
      Tolerance_20    =   2
      Animator_19     =   0
      Blinker_19      =   0
      varVarType_14   =   5
      Bindings_1      =   21
      ClassName_21    =   "CCWBindingHolderArray"
      Editor_21       =   22
      ClassName_22    =   "CCWBindingHolderArrayEditor"
      Owner_22        =   1
      Style_1         =   4
      mechAction_1    =   1
      BGImg_1         =   23
      ClassName_23    =   "CCWDrawObj"
      opts_23         =   62
      Image_23        =   24
      ClassName_24    =   "CCWPictImage"
      opts_24         =   1280
      Rows_24         =   1
      Cols_24         =   1
      Pict_24         =   286
      F_24            =   -2147483633
      B_24            =   -2147483633
      ColorReplaceWith_24=   8421504
      ColorReplace_24 =   8421504
      Tolerance_24    =   2
      Animator_23     =   0
      Blinker_23      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   12
      Array[1]_1      =   19
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   9
      Animator_25     =   0
      Blinker_25      =   0
      Array[5]_1      =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      Image_26        =   11
      Animator_26     =   0
      Blinker_26      =   0
      Label_1         =   27
      ClassName_27    =   "CCWDrawObj"
      opts_27         =   62
      C[0]_27         =   -2147483640
      Image_27        =   6
      Animator_27     =   0
      Blinker_27      =   0
   End
   Begin CWUIControlsLib.CWButton LEDButton 
      Height          =   252
      HelpContextID   =   330
      Index           =   1
      Left            =   4800
      TabIndex        =   31
      Top             =   5160
      Width           =   252
      _Version        =   196609
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.9
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   6686
      C[0]_1          =   14737632
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   286
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   7
      ClassName_7     =   "CCWBlinker"
      Interval_7      =   600
      Enable_7        =   -1  'True
      EventID_7       =   7
      Obj_7           =   5
      list[3]_4       =   8
      ClassName_8     =   "CCWDrawObj"
      opts_8          =   60
      C[0]_8          =   -2147483640
      C[1]_8          =   -2147483640
      Image_8         =   9
      ClassName_9     =   "CCWTextImage"
      style_9         =   1076101120
      font_9          =   0
      Animator_8      =   0
      Blinker_8       =   0
      list[2]_4       =   10
      ClassName_10    =   "CCWDrawObj"
      opts_10         =   62
      C[0]_10         =   -2147483640
      C[1]_10         =   -2147483640
      Image_10        =   11
      ClassName_11    =   "CCWTextImage"
      font_11         =   0
      Animator_10     =   0
      Blinker_10      =   0
      list[1]_4       =   12
      ClassName_12    =   "CCWDrawObj"
      opts_12         =   62
      C[0]_12         =   12632256
      C[1]_12         =   12632256
      Image_12        =   13
      ClassName_13    =   "CCWPictImage"
      opts_13         =   1280
      Rows_13         =   1
      Cols_13         =   1
      Pict_13         =   73
      F_13            =   12632256
      B_13            =   12632256
      ColorReplaceWith_13=   8421504
      ColorReplace_13 =   8421504
      Tolerance_13    =   2
      Animator_12     =   0
      Blinker_12      =   0
      varVarType_3    =   5
      Array[1]_2      =   14
      ClassName_14    =   "CCWEnumElt"
      opts_14         =   1
      Name_14         =   "On"
      frame_14        =   286
      DrawList_14     =   15
      ClassName_15    =   "CDrawList"
      count_15        =   4
      list[4]_15      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   -2147483640
      C[1]_16         =   -2147483640
      Image_16        =   6
      Animator_16     =   0
      Blinker_16      =   0
      list[3]_15      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   62
      C[0]_17         =   -2147483640
      C[1]_17         =   -2147483640
      Image_17        =   9
      Animator_17     =   0
      Blinker_17      =   0
      list[2]_15      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   60
      C[0]_18         =   -2147483640
      C[1]_18         =   -2147483640
      Image_18        =   11
      Animator_18     =   0
      Blinker_18      =   0
      list[1]_15      =   19
      ClassName_19    =   "CCWDrawObj"
      opts_19         =   62
      C[0]_19         =   65535
      C[1]_19         =   65535
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   65535
      B_20            =   65535
      ColorReplaceWith_20=   8421504
      ColorReplace_20 =   8421504
      Tolerance_20    =   2
      Animator_19     =   0
      Blinker_19      =   0
      varVarType_14   =   5
      Bindings_1      =   21
      ClassName_21    =   "CCWBindingHolderArray"
      Editor_21       =   22
      ClassName_22    =   "CCWBindingHolderArrayEditor"
      Owner_22        =   1
      Style_1         =   4
      mechAction_1    =   1
      BGImg_1         =   23
      ClassName_23    =   "CCWDrawObj"
      opts_23         =   62
      Image_23        =   24
      ClassName_24    =   "CCWPictImage"
      opts_24         =   1280
      Rows_24         =   1
      Cols_24         =   1
      Pict_24         =   286
      F_24            =   -2147483633
      B_24            =   -2147483633
      ColorReplaceWith_24=   8421504
      ColorReplace_24 =   8421504
      Tolerance_24    =   2
      Animator_23     =   0
      Blinker_23      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   12
      Array[1]_1      =   19
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   9
      Animator_25     =   0
      Blinker_25      =   0
      Array[5]_1      =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      Image_26        =   11
      Animator_26     =   0
      Blinker_26      =   0
      Label_1         =   27
      ClassName_27    =   "CCWDrawObj"
      opts_27         =   62
      C[0]_27         =   -2147483640
      Image_27        =   6
      Animator_27     =   0
      Blinker_27      =   0
   End
   Begin CWUIControlsLib.CWButton LEDButton 
      Height          =   252
      HelpContextID   =   330
      Index           =   6
      Left            =   9360
      TabIndex        =   34
      Top             =   5160
      Width           =   252
      _Version        =   196609
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.9
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   6686
      C[0]_1          =   14737632
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   286
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   7
      ClassName_7     =   "CCWBlinker"
      Interval_7      =   600
      Enable_7        =   -1  'True
      EventID_7       =   7
      Obj_7           =   5
      list[3]_4       =   8
      ClassName_8     =   "CCWDrawObj"
      opts_8          =   60
      C[0]_8          =   -2147483640
      C[1]_8          =   -2147483640
      Image_8         =   9
      ClassName_9     =   "CCWTextImage"
      style_9         =   1076101120
      font_9          =   0
      Animator_8      =   0
      Blinker_8       =   0
      list[2]_4       =   10
      ClassName_10    =   "CCWDrawObj"
      opts_10         =   62
      C[0]_10         =   -2147483640
      C[1]_10         =   -2147483640
      Image_10        =   11
      ClassName_11    =   "CCWTextImage"
      font_11         =   0
      Animator_10     =   0
      Blinker_10      =   0
      list[1]_4       =   12
      ClassName_12    =   "CCWDrawObj"
      opts_12         =   62
      C[0]_12         =   12632256
      C[1]_12         =   12632256
      Image_12        =   13
      ClassName_13    =   "CCWPictImage"
      opts_13         =   1280
      Rows_13         =   1
      Cols_13         =   1
      Pict_13         =   73
      F_13            =   12632256
      B_13            =   12632256
      ColorReplaceWith_13=   8421504
      ColorReplace_13 =   8421504
      Tolerance_13    =   2
      Animator_12     =   0
      Blinker_12      =   0
      varVarType_3    =   5
      Array[1]_2      =   14
      ClassName_14    =   "CCWEnumElt"
      opts_14         =   1
      Name_14         =   "On"
      frame_14        =   286
      DrawList_14     =   15
      ClassName_15    =   "CDrawList"
      count_15        =   4
      list[4]_15      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   -2147483640
      C[1]_16         =   -2147483640
      Image_16        =   6
      Animator_16     =   0
      Blinker_16      =   0
      list[3]_15      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   62
      C[0]_17         =   -2147483640
      C[1]_17         =   -2147483640
      Image_17        =   9
      Animator_17     =   0
      Blinker_17      =   0
      list[2]_15      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   60
      C[0]_18         =   -2147483640
      C[1]_18         =   -2147483640
      Image_18        =   11
      Animator_18     =   0
      Blinker_18      =   0
      list[1]_15      =   19
      ClassName_19    =   "CCWDrawObj"
      opts_19         =   62
      C[0]_19         =   49152
      C[1]_19         =   49152
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   49152
      B_20            =   49152
      ColorReplaceWith_20=   8421504
      ColorReplace_20 =   8421504
      Tolerance_20    =   2
      Animator_19     =   0
      Blinker_19      =   0
      varVarType_14   =   5
      Bindings_1      =   21
      ClassName_21    =   "CCWBindingHolderArray"
      Editor_21       =   22
      ClassName_22    =   "CCWBindingHolderArrayEditor"
      Owner_22        =   1
      Style_1         =   4
      mechAction_1    =   1
      BGImg_1         =   23
      ClassName_23    =   "CCWDrawObj"
      opts_23         =   62
      Image_23        =   24
      ClassName_24    =   "CCWPictImage"
      opts_24         =   1280
      Rows_24         =   1
      Cols_24         =   1
      Pict_24         =   286
      F_24            =   -2147483633
      B_24            =   -2147483633
      ColorReplaceWith_24=   8421504
      ColorReplace_24 =   8421504
      Tolerance_24    =   2
      Animator_23     =   0
      Blinker_23      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   12
      Array[1]_1      =   19
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   9
      Animator_25     =   0
      Blinker_25      =   0
      Array[5]_1      =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      Image_26        =   11
      Animator_26     =   0
      Blinker_26      =   0
      Label_1         =   27
      ClassName_27    =   "CCWDrawObj"
      opts_27         =   62
      C[0]_27         =   -2147483640
      Image_27        =   6
      Animator_27     =   0
      Blinker_27      =   0
   End
   Begin CWUIControlsLib.CWButton LEDButton 
      Height          =   252
      HelpContextID   =   330
      Index           =   5
      Left            =   9720
      TabIndex        =   35
      Top             =   5160
      Width           =   252
      _Version        =   196609
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.9
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   6686
      C[0]_1          =   14737632
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   286
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   7
      ClassName_7     =   "CCWBlinker"
      Interval_7      =   600
      Enable_7        =   -1  'True
      EventID_7       =   7
      Obj_7           =   5
      list[3]_4       =   8
      ClassName_8     =   "CCWDrawObj"
      opts_8          =   60
      C[0]_8          =   -2147483640
      C[1]_8          =   -2147483640
      Image_8         =   9
      ClassName_9     =   "CCWTextImage"
      style_9         =   1076101120
      font_9          =   0
      Animator_8      =   0
      Blinker_8       =   0
      list[2]_4       =   10
      ClassName_10    =   "CCWDrawObj"
      opts_10         =   62
      C[0]_10         =   -2147483640
      C[1]_10         =   -2147483640
      Image_10        =   11
      ClassName_11    =   "CCWTextImage"
      font_11         =   0
      Animator_10     =   0
      Blinker_10      =   0
      list[1]_4       =   12
      ClassName_12    =   "CCWDrawObj"
      opts_12         =   62
      C[0]_12         =   12632256
      C[1]_12         =   12632256
      Image_12        =   13
      ClassName_13    =   "CCWPictImage"
      opts_13         =   1280
      Rows_13         =   1
      Cols_13         =   1
      Pict_13         =   73
      F_13            =   12632256
      B_13            =   12632256
      ColorReplaceWith_13=   8421504
      ColorReplace_13 =   8421504
      Tolerance_13    =   2
      Animator_12     =   0
      Blinker_12      =   0
      varVarType_3    =   5
      Array[1]_2      =   14
      ClassName_14    =   "CCWEnumElt"
      opts_14         =   1
      Name_14         =   "On"
      frame_14        =   286
      DrawList_14     =   15
      ClassName_15    =   "CDrawList"
      count_15        =   4
      list[4]_15      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   -2147483640
      C[1]_16         =   -2147483640
      Image_16        =   6
      Animator_16     =   0
      Blinker_16      =   0
      list[3]_15      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   62
      C[0]_17         =   -2147483640
      C[1]_17         =   -2147483640
      Image_17        =   9
      Animator_17     =   0
      Blinker_17      =   0
      list[2]_15      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   60
      C[0]_18         =   -2147483640
      C[1]_18         =   -2147483640
      Image_18        =   11
      Animator_18     =   0
      Blinker_18      =   0
      list[1]_15      =   19
      ClassName_19    =   "CCWDrawObj"
      opts_19         =   62
      C[0]_19         =   65535
      C[1]_19         =   65535
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   65535
      B_20            =   65535
      ColorReplaceWith_20=   8421504
      ColorReplace_20 =   8421504
      Tolerance_20    =   2
      Animator_19     =   0
      Blinker_19      =   0
      varVarType_14   =   5
      Bindings_1      =   21
      ClassName_21    =   "CCWBindingHolderArray"
      Editor_21       =   22
      ClassName_22    =   "CCWBindingHolderArrayEditor"
      Owner_22        =   1
      Style_1         =   4
      mechAction_1    =   1
      BGImg_1         =   23
      ClassName_23    =   "CCWDrawObj"
      opts_23         =   62
      Image_23        =   24
      ClassName_24    =   "CCWPictImage"
      opts_24         =   1280
      Rows_24         =   1
      Cols_24         =   1
      Pict_24         =   286
      F_24            =   -2147483633
      B_24            =   -2147483633
      ColorReplaceWith_24=   8421504
      ColorReplace_24 =   8421504
      Tolerance_24    =   2
      Animator_23     =   0
      Blinker_23      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   12
      Array[1]_1      =   19
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   9
      Animator_25     =   0
      Blinker_25      =   0
      Array[5]_1      =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      Image_26        =   11
      Animator_26     =   0
      Blinker_26      =   0
      Label_1         =   27
      ClassName_27    =   "CCWDrawObj"
      opts_27         =   62
      C[0]_27         =   -2147483640
      Image_27        =   6
      Animator_27     =   0
      Blinker_27      =   0
   End
   Begin CWUIControlsLib.CWButton LEDButton 
      Height          =   252
      HelpContextID   =   330
      Index           =   4
      Left            =   10080
      TabIndex        =   36
      Top             =   5160
      Width           =   252
      _Version        =   196609
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.9
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   6670
      C[0]_1          =   14737632
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   286
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   7
      ClassName_7     =   "CCWBlinker"
      Interval_7      =   600
      Enable_7        =   -1  'True
      EventID_7       =   7
      Obj_7           =   5
      list[3]_4       =   8
      ClassName_8     =   "CCWDrawObj"
      opts_8          =   60
      C[0]_8          =   -2147483640
      C[1]_8          =   -2147483640
      Image_8         =   9
      ClassName_9     =   "CCWTextImage"
      style_9         =   1076101120
      font_9          =   0
      Animator_8      =   0
      Blinker_8       =   0
      list[2]_4       =   10
      ClassName_10    =   "CCWDrawObj"
      opts_10         =   62
      C[0]_10         =   -2147483640
      C[1]_10         =   -2147483640
      Image_10        =   11
      ClassName_11    =   "CCWTextImage"
      font_11         =   0
      Animator_10     =   0
      Blinker_10      =   0
      list[1]_4       =   12
      ClassName_12    =   "CCWDrawObj"
      opts_12         =   62
      C[0]_12         =   12632256
      C[1]_12         =   12632256
      Image_12        =   13
      ClassName_13    =   "CCWPictImage"
      opts_13         =   1280
      Rows_13         =   1
      Cols_13         =   1
      Pict_13         =   73
      F_13            =   12632256
      B_13            =   12632256
      ColorReplaceWith_13=   8421504
      ColorReplace_13 =   8421504
      Tolerance_13    =   2
      Animator_12     =   0
      Blinker_12      =   0
      varVarType_3    =   5
      Array[1]_2      =   14
      ClassName_14    =   "CCWEnumElt"
      opts_14         =   1
      Name_14         =   "On"
      frame_14        =   286
      DrawList_14     =   15
      ClassName_15    =   "CDrawList"
      count_15        =   4
      list[4]_15      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   -2147483640
      C[1]_16         =   -2147483640
      Image_16        =   6
      Animator_16     =   0
      Blinker_16      =   0
      list[3]_15      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   62
      C[0]_17         =   -2147483640
      C[1]_17         =   -2147483640
      Image_17        =   9
      Animator_17     =   0
      Blinker_17      =   0
      list[2]_15      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   60
      C[0]_18         =   -2147483640
      C[1]_18         =   -2147483640
      Image_18        =   11
      Animator_18     =   0
      Blinker_18      =   0
      list[1]_15      =   19
      ClassName_19    =   "CCWDrawObj"
      opts_19         =   62
      C[0]_19         =   65535
      C[1]_19         =   65535
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   65535
      B_20            =   65535
      ColorReplaceWith_20=   8421504
      ColorReplace_20 =   8421504
      Tolerance_20    =   2
      Animator_19     =   0
      Blinker_19      =   0
      varVarType_14   =   5
      Bindings_1      =   21
      ClassName_21    =   "CCWBindingHolderArray"
      Editor_21       =   22
      ClassName_22    =   "CCWBindingHolderArrayEditor"
      Owner_22        =   1
      Style_1         =   4
      mechAction_1    =   1
      BGImg_1         =   23
      ClassName_23    =   "CCWDrawObj"
      opts_23         =   62
      Image_23        =   24
      ClassName_24    =   "CCWPictImage"
      opts_24         =   1280
      Rows_24         =   1
      Cols_24         =   1
      Pict_24         =   286
      F_24            =   -2147483633
      B_24            =   -2147483633
      ColorReplaceWith_24=   8421504
      ColorReplace_24 =   8421504
      Tolerance_24    =   2
      Animator_23     =   0
      Blinker_23      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   12
      Array[1]_1      =   19
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   9
      Animator_25     =   0
      Blinker_25      =   0
      Array[5]_1      =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      Image_26        =   11
      Animator_26     =   0
      Blinker_26      =   0
      Label_1         =   27
      ClassName_27    =   "CCWDrawObj"
      opts_27         =   62
      C[0]_27         =   -2147483640
      Image_27        =   6
      Animator_27     =   0
      Blinker_27      =   0
   End
   Begin VB.Frame Frame3 
      Caption         =   "Rückkehr zu Abschnitt 1"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   1335
      Left            =   12600
      TabIndex        =   44
      Top             =   1560
      Width           =   1935
   End
   Begin VB.Frame Frame1 
      BackColor       =   &H00E0E0E0&
      Caption         =   "U-PHASE"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   1092
      Index           =   1
      Left            =   5520
      TabIndex        =   48
      Top             =   4560
      Width           =   5052
   End
   Begin CWUIControlsLib.CWNumEdit num1 
      Height          =   288
      HelpContextID   =   320
      Index           =   3
      Left            =   3000
      TabIndex        =   28
      Top             =   5160
      Visible         =   0   'False
      Width           =   852
      _Version        =   196609
      _ExtentX        =   1503
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
      opts_1          =   327742
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   50
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   50
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin VB.Frame Frame1 
      BackColor       =   &H00E0E0E0&
      Caption         =   "I-PHASE"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   1092
      Index           =   0
      Left            =   240
      TabIndex        =   46
      Top             =   4560
      Width           =   5052
   End
   Begin CWUIControlsLib.CWButton CWButton2 
      Height          =   375
      Index           =   5
      Left            =   15720
      TabIndex        =   54
      Top             =   7920
      Visible         =   0   'False
      Width           =   375
      _Version        =   196609
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.86
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "And"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "Or"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin CWUIControlsLib.CWButton CWButton2 
      Height          =   375
      Index           =   2
      Left            =   15720
      TabIndex        =   55
      Top             =   6840
      Visible         =   0   'False
      Width           =   375
      _Version        =   196609
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.86
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "And"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "Or"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin CWUIControlsLib.CWButton CWButton2 
      Height          =   375
      Index           =   3
      Left            =   15720
      TabIndex        =   56
      Top             =   7200
      Visible         =   0   'False
      Width           =   375
      _Version        =   196609
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   7.86
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196609
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "And"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "Or"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin CWUIControlsLib.CWButton AndOrButton 
      Height          =   372
      HelpContextID   =   350
      Index           =   1
      Left            =   9840
      TabIndex        =   41
      Top             =   6360
      Visible         =   0   'False
      Width           =   612
      _Version        =   196609
      _ExtentX        =   1080
      _ExtentY        =   656
      _StockProps     =   68
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
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "UND"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "ODER"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin CWUIControlsLib.CWButton AndOrButton 
      Height          =   372
      HelpContextID   =   350
      Index           =   2
      Left            =   9840
      TabIndex        =   43
      Top             =   6720
      Visible         =   0   'False
      Width           =   612
      _Version        =   196609
      _ExtentX        =   1080
      _ExtentY        =   656
      _StockProps     =   68
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
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "UND"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "ODER"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   1
      Left            =   14160
      TabIndex        =   66
      Top             =   6720
      Visible         =   0   'False
      Width           =   855
      _Version        =   196609
      _ExtentX        =   1508
      _ExtentY        =   450
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   2
      Left            =   14160
      TabIndex        =   67
      Top             =   7080
      Visible         =   0   'False
      Width           =   855
      _Version        =   196609
      _ExtentX        =   1508
      _ExtentY        =   450
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   3
      Left            =   14160
      TabIndex        =   68
      Top             =   7440
      Visible         =   0   'False
      Width           =   855
      _Version        =   196609
      _ExtentX        =   1508
      _ExtentY        =   450
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   4
      Left            =   14160
      TabIndex        =   69
      Top             =   7800
      Visible         =   0   'False
      Width           =   855
      _Version        =   196609
      _ExtentX        =   1508
      _ExtentY        =   450
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit BedVorwWert 
      Height          =   288
      HelpContextID   =   350
      Index           =   1
      Left            =   8880
      TabIndex        =   38
      Top             =   6240
      Visible         =   0   'False
      Width           =   972
      _Version        =   196609
      _ExtentX        =   1714
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit BedVorwWert 
      Height          =   288
      HelpContextID   =   350
      Index           =   2
      Left            =   8880
      TabIndex        =   39
      Top             =   6600
      Visible         =   0   'False
      Width           =   972
      _Version        =   196609
      _ExtentX        =   1714
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit BedVorwWert 
      Height          =   288
      HelpContextID   =   350
      Index           =   3
      Left            =   8880
      TabIndex        =   40
      Top             =   6960
      Visible         =   0   'False
      Width           =   972
      _Version        =   196609
      _ExtentX        =   1714
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin MSCommLib.MSComm MSComm1 
      Left            =   12960
      Top             =   7320
      _ExtentX        =   995
      _ExtentY        =   995
      _Version        =   393216
      DTREnable       =   -1  'True
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   100
      Index           =   0
      Left            =   4560
      TabIndex        =   6
      Top             =   720
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      Value_Val_1     =   24
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMin_Val_1  =   12
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   600
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   100
      Index           =   1
      Left            =   4560
      TabIndex        =   7
      Top             =   1080
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.01
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   0.01
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMin_Val_1  =   -1
      RangeMaxVarType_1=   5
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   100
      Index           =   2
      Left            =   4560
      TabIndex        =   8
      Top             =   1440
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.5
      ValueVarType_1  =   5
      Value_Val_1     =   25
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   50
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   100
      Index           =   3
      Left            =   12480
      TabIndex        =   92
      Top             =   5640
      Width           =   1308
      _Version        =   196609
      _ExtentX        =   2302
      _ExtentY        =   503
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
      opts_1          =   327742
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      Value_Val_1     =   3680
      IncValueVarType_1=   5
      IncValue_Val_1  =   10
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   100
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   59999
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   100
      Index           =   4
      Left            =   4560
      TabIndex        =   12
      Top             =   2280
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      Value_Val_1     =   16
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   50
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   100
      Index           =   5
      Left            =   4560
      TabIndex        =   14
      Top             =   3120
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      Value_Val_1     =   -4
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMin_Val_1  =   -10
      RangeMaxVarType_1=   5
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   100
      Index           =   6
      Left            =   4560
      TabIndex        =   15
      Top             =   3480
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.5
      ValueVarType_1  =   5
      Value_Val_1     =   50
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMin_Val_1  =   20
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   60
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   200
      Index           =   7
      Left            =   9480
      TabIndex        =   19
      Top             =   2400
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.5
      ValueVarType_1  =   5
      Value_Val_1     =   55
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   70
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   200
      Index           =   8
      Left            =   9480
      TabIndex        =   20
      Top             =   2760
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      Value_Val_1     =   400
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   720
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   200
      Index           =   9
      Left            =   12480
      TabIndex        =   21
      TabStop         =   0   'False
      Top             =   6000
      Width           =   1188
      _Version        =   196609
      _ExtentX        =   2090
      _ExtentY        =   503
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
      opts_1          =   327742
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   2999
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   200
      Index           =   10
      Left            =   9480
      TabIndex        =   22
      Top             =   3120
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   999
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   200
      Index           =   11
      Left            =   9480
      TabIndex        =   23
      Top             =   3480
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   999
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   170
      Index           =   12
      Left            =   9480
      TabIndex        =   17
      Top             =   1080
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      Value_Val_1     =   6
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMin_Val_1  =   1
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   12
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   288
      HelpContextID   =   170
      Index           =   13
      Left            =   9480
      TabIndex        =   18
      Top             =   1440
      Width           =   1068
      _Version        =   196609
      _ExtentX        =   1884
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
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      dMax_3          =   10
      discInterval_3  =   0.1
      ValueVarType_1  =   5
      Value_Val_1     =   10
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMin_Val_1  =   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   16
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit BedVorwWert 
      Height          =   288
      HelpContextID   =   350
      Index           =   4
      Left            =   8880
      TabIndex        =   106
      Top             =   7320
      Visible         =   0   'False
      Width           =   972
      _Version        =   196609
      _ExtentX        =   1714
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit BedVorwWert 
      Height          =   288
      HelpContextID   =   350
      Index           =   5
      Left            =   8880
      TabIndex        =   107
      Top             =   7680
      Visible         =   0   'False
      Width           =   972
      _Version        =   196609
      _ExtentX        =   1714
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
      opts_1          =   393278
      C[0]_1          =   16777215
      BorderStyle_1   =   1
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.00"
      scale_1         =   3
      ClassName_3     =   "CCWScale"
      opts_3          =   65536
      dMax_3          =   10
      discInterval_3  =   1
      ValueVarType_1  =   5
      IncValueVarType_1=   5
      IncValue_Val_1  =   1
      AccelIncVarType_1=   5
      AccelInc_Val_1  =   5
      RangeMinVarType_1=   5
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   100
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWButton AndOrButton 
      Height          =   372
      HelpContextID   =   350
      Index           =   3
      Left            =   9840
      TabIndex        =   108
      Top             =   7080
      Visible         =   0   'False
      Width           =   612
      _Version        =   196609
      _ExtentX        =   1080
      _ExtentY        =   656
      _StockProps     =   68
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
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "UND"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "ODER"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin CWUIControlsLib.CWButton AndOrButton 
      Height          =   372
      HelpContextID   =   350
      Index           =   4
      Left            =   9840
      TabIndex        =   109
      Top             =   7440
      Visible         =   0   'False
      Width           =   612
      _Version        =   196609
      _ExtentX        =   1080
      _ExtentY        =   656
      _StockProps     =   68
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
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2622
      C[0]_1          =   -2147483643
      Enum_1          =   2
      ClassName_2     =   "CCWEnum"
      Array_2         =   2
      Editor_2        =   0
      Array[0]_2      =   3
      ClassName_3     =   "CCWEnumElt"
      opts_3          =   1
      Name_3          =   "Off"
      frame_3         =   288
      DrawList_3      =   4
      ClassName_4     =   "CDrawList"
      count_4         =   4
      list[4]_4       =   5
      ClassName_5     =   "CCWDrawObj"
      opts_5          =   62
      C[0]_5          =   -2147483640
      C[1]_5          =   -2147483640
      Image_5         =   6
      ClassName_6     =   "CCWTextImage"
      style_6         =   16777217
      font_6          =   0
      Animator_5      =   0
      Blinker_5       =   0
      list[3]_4       =   7
      ClassName_7     =   "CCWDrawObj"
      opts_7          =   60
      C[0]_7          =   255
      C[1]_7          =   255
      Image_7         =   8
      ClassName_8     =   "CCWTextImage"
      szText_8        =   "UND"
      style_8         =   16777217
      font_8          =   0
      Animator_7      =   0
      Blinker_7       =   0
      list[2]_4       =   9
      ClassName_9     =   "CCWDrawObj"
      opts_9          =   62
      C[0]_9          =   16711680
      C[1]_9          =   16711680
      Image_9         =   10
      ClassName_10    =   "CCWTextImage"
      szText_10       =   "ODER"
      style_10        =   16777217
      font_10         =   0
      Animator_9      =   0
      Blinker_9       =   0
      list[1]_4       =   11
      ClassName_11    =   "CCWDrawObj"
      opts_11         =   62
      Image_11        =   12
      ClassName_12    =   "CCWPictImage"
      opts_12         =   1280
      Rows_12         =   1
      Cols_12         =   1
      Pict_12         =   2
      F_12            =   -2147483633
      B_12            =   -2147483633
      ColorReplaceWith_12=   8421504
      ColorReplace_12 =   8421504
      Tolerance_12    =   2
      Animator_11     =   0
      Blinker_11      =   0
      varVarType_3    =   5
      Array[1]_2      =   13
      ClassName_13    =   "CCWEnumElt"
      opts_13         =   1
      Name_13         =   "On"
      frame_13        =   287
      DrawList_13     =   14
      ClassName_14    =   "CDrawList"
      count_14        =   4
      list[4]_14      =   15
      ClassName_15    =   "CCWDrawObj"
      opts_15         =   62
      C[0]_15         =   -2147483640
      C[1]_15         =   -2147483640
      Image_15        =   6
      Animator_15     =   0
      Blinker_15      =   0
      list[3]_14      =   16
      ClassName_16    =   "CCWDrawObj"
      opts_16         =   62
      C[0]_16         =   255
      C[1]_16         =   255
      Image_16        =   8
      Animator_16     =   0
      Blinker_16      =   0
      list[2]_14      =   17
      ClassName_17    =   "CCWDrawObj"
      opts_17         =   60
      C[0]_17         =   16711680
      C[1]_17         =   16711680
      Image_17        =   10
      Animator_17     =   0
      Blinker_17      =   0
      list[1]_14      =   18
      ClassName_18    =   "CCWDrawObj"
      opts_18         =   62
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   2
      F_19            =   -2147483633
      B_19            =   -2147483633
      ColorReplaceWith_19=   8421504
      ColorReplace_19 =   8421504
      Tolerance_19    =   2
      Animator_18     =   0
      Blinker_18      =   0
      varVarType_13   =   5
      Bindings_1      =   20
      ClassName_20    =   "CCWBindingHolderArray"
      Editor_20       =   21
      ClassName_21    =   "CCWBindingHolderArrayEditor"
      Owner_21        =   1
      Style_1         =   12
      frameStyle_1    =   1
      mechAction_1    =   1
      BGImg_1         =   22
      ClassName_22    =   "CCWDrawObj"
      opts_22         =   62
      Image_22        =   23
      ClassName_23    =   "CCWPictImage"
      opts_23         =   1280
      Rows_23         =   1
      Cols_23         =   1
      Pict_23         =   286
      F_23            =   -2147483633
      B_23            =   -2147483633
      ColorReplaceWith_23=   8421504
      ColorReplace_23 =   8421504
      Tolerance_23    =   2
      Animator_22     =   0
      Blinker_22      =   0
      Array_1         =   6
      Editor_1        =   0
      Array[0]_1      =   11
      Array[1]_1      =   18
      Array[2]_1      =   0
      Array[3]_1      =   0
      Array[4]_1      =   24
      ClassName_24    =   "CCWDrawObj"
      opts_24         =   62
      Image_24        =   8
      Animator_24     =   0
      Blinker_24      =   0
      Array[5]_1      =   25
      ClassName_25    =   "CCWDrawObj"
      opts_25         =   62
      Image_25        =   10
      Animator_25     =   0
      Blinker_25      =   0
      Label_1         =   26
      ClassName_26    =   "CCWDrawObj"
      opts_26         =   62
      C[0]_26         =   -2147483640
      Image_26        =   6
      Animator_26     =   0
      Blinker_26      =   0
   End
   Begin VB.Label BedVorwText 
      BackColor       =   &H00C0C0C0&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "5"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   5
      Left            =   4920
      TabIndex        =   105
      ToolTipText     =   "Doppelklick auf Bedingung entfernt diese"
      Top             =   7680
      Visible         =   0   'False
      Width           =   3852
   End
   Begin VB.Label BedVorwText 
      BackColor       =   &H00C0C0C0&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "4"
      DataField       =   "3"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   4
      Left            =   4920
      TabIndex        =   104
      ToolTipText     =   "Doppelklick auf Bedingung entfernt diese"
      Top             =   7320
      Visible         =   0   'False
      Width           =   3852
   End
   Begin VB.Label LblDebug 
      BackColor       =   &H00E0E0E0&
      Caption         =   "DEBUG V3.0 Beta 4"
      ForeColor       =   &H000000FF&
      Height          =   252
      Left            =   7320
      TabIndex        =   103
      Top             =   120
      Width           =   2412
   End
   Begin VB.Label LabelBetrArt 
      AutoSize        =   -1  'True
      BackColor       =   &H00E0E0E0&
      Caption         =   " Betriebsart     "
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   192
      Left            =   6120
      TabIndex        =   101
      Top             =   360
      Width           =   996
   End
   Begin VB.Shape Shape5 
      BackColor       =   &H000080FF&
      BorderColor     =   &H00000000&
      Height          =   1332
      Left            =   6000
      Top             =   480
      Width           =   4692
   End
   Begin VB.Label LabelLadeKennl 
      AutoSize        =   -1  'True
      BackColor       =   &H00E0E0E0&
      Caption         =   " Ladekennlinie"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   192
      Left            =   240
      TabIndex        =   100
      Top             =   4080
      Width           =   1044
   End
   Begin VB.Label LblMitGeraet 
      BackColor       =   &H00E0E0E0&
      Caption         =   "mit Gerät"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Left            =   3960
      TabIndex        =   2
      Top             =   120
      Width           =   972
   End
   Begin VB.Label status_fenster 
      Alignment       =   1  'Rechts
      BackColor       =   &H00E0E0E0&
      Caption         =   "Verbunden"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      ForeColor       =   &H00008000&
      Height          =   252
      Index           =   1
      Left            =   2640
      TabIndex        =   1
      Top             =   120
      Width           =   1212
   End
   Begin VB.Label LabelSicherAbschalt 
      AutoSize        =   -1  'True
      BackColor       =   &H00E0E0E0&
      Caption         =   " Abschaltung in jedem Fall, wenn:"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   192
      Left            =   6120
      TabIndex        =   95
      Top             =   2040
      Width           =   2352
   End
   Begin VB.Shape Shape6 
      BackColor       =   &H00000000&
      BorderColor     =   &H00000000&
      Height          =   1692
      Left            =   6000
      Top             =   2160
      Width           =   4692
   End
   Begin VB.Label LabelAllg 
      AutoSize        =   -1  'True
      BackColor       =   &H00E0E0E0&
      Caption         =   " Allgemeine Einstellungen"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   192
      Left            =   240
      TabIndex        =   94
      Top             =   360
      Width           =   1848
   End
   Begin VB.Shape ShapeAllg 
      Height          =   3372
      Left            =   120
      Top             =   480
      Width           =   5652
   End
   Begin VB.Label Label5 
      Caption         =   " Wechsel - Bedingung:"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Left            =   2640
      TabIndex        =   93
      Top             =   10800
      Width           =   1695
   End
   Begin VB.Label status_fenster 
      Alignment       =   1  'Rechts
      BackColor       =   &H00E0E0E0&
      Caption         =   "Nicht verbunden"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      ForeColor       =   &H000000C0&
      Height          =   252
      Index           =   0
      Left            =   2280
      TabIndex        =   0
      Top             =   120
      Width           =   1572
   End
   Begin VB.Shape AbschnitteRahmen 
      BorderColor     =   &H00000000&
      Height          =   3972
      Left            =   120
      Top             =   4200
      Width           =   10572
   End
   Begin VB.Label LabelWechselBed 
      AutoSize        =   -1  'True
      BackColor       =   &H00E0E0E0&
      Caption         =   " >>> Zum nächsten Abschnitt wechseln, wenn: >>>"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   192
      Left            =   360
      TabIndex        =   78
      Top             =   5880
      Width           =   3480
   End
   Begin VB.Label Label1 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      Caption         =   "<<<<< Zum vorherigen Abschnitt, wenn:"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Left            =   12600
      TabIndex        =   77
      Top             =   3480
      Width           =   1452
   End
   Begin VB.Shape Shape2 
      BorderColor     =   &H00000000&
      Height          =   2077
      Left            =   240
      Top             =   6000
      Width           =   10332
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "4"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   4
      Left            =   12600
      TabIndex        =   61
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   4920
      Visible         =   0   'False
      Width           =   1452
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "3"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   3
      Left            =   12600
      TabIndex        =   60
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   4560
      Visible         =   0   'False
      Width           =   1452
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "2"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   2
      Left            =   12600
      TabIndex        =   59
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   4200
      Visible         =   0   'False
      Width           =   1452
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "1"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   1
      Left            =   12600
      TabIndex        =   58
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   3840
      Visible         =   0   'False
      Width           =   1452
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "5"
      Enabled         =   0   'False
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   5
      Left            =   12600
      TabIndex        =   57
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   5280
      Visible         =   0   'False
      Width           =   1452
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   4
      Left            =   15000
      TabIndex        =   74
      Top             =   7800
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   3
      Left            =   15000
      TabIndex        =   73
      Top             =   7440
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   2
      Left            =   15000
      TabIndex        =   72
      Top             =   7080
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   1
      Left            =   15000
      TabIndex        =   71
      Top             =   6720
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Index           =   5
      Left            =   15000
      TabIndex        =   70
      Top             =   8160
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label BedVorwText 
      BackColor       =   &H00C0C0C0&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "3"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   3
      Left            =   4920
      TabIndex        =   64
      ToolTipText     =   "Doppelklick auf Bedingung entfernt diese"
      Top             =   6960
      Visible         =   0   'False
      Width           =   3852
   End
   Begin VB.Label BedVorwText 
      BackColor       =   &H00C0C0C0&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "2"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   2
      Left            =   4920
      TabIndex        =   63
      ToolTipText     =   "Doppelklick auf Bedingung entfernt diese"
      Top             =   6600
      Visible         =   0   'False
      Width           =   3852
   End
   Begin VB.Label BedVorwText 
      BackColor       =   &H00C0C0C0&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "1"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   252
      Index           =   1
      Left            =   4920
      TabIndex        =   62
      ToolTipText     =   "Doppelklick auf Bedingung entfernt diese"
      Top             =   6240
      Visible         =   0   'False
      Width           =   3852
   End
   Begin VB.Menu Profile_Mnu 
      Caption         =   "&Kennlinie"
      Begin VB.Menu new_profile 
         Caption         =   "&Neu"
         Shortcut        =   ^N
      End
      Begin VB.Menu load_profile 
         Caption         =   "von Datei &Laden"
         Shortcut        =   ^O
      End
      Begin VB.Menu save_profile 
         Caption         =   "in Datei &Speichern"
         Shortcut        =   ^S
      End
      Begin VB.Menu save_as 
         Caption         =   "Speichern unter"
         Enabled         =   0   'False
         Visible         =   0   'False
      End
      Begin VB.Menu ligne_blank 
         Caption         =   "-"
      End
      Begin VB.Menu Profile_from 
         Caption         =   "&vom NLG empfangen"
         Shortcut        =   {F6}
      End
      Begin VB.Menu Profile_to 
         Caption         =   "&zum NLG senden"
         Shortcut        =   {F7}
      End
      Begin VB.Menu blanc_0 
         Caption         =   "-"
      End
      Begin VB.Menu Quitter 
         Caption         =   "&Beenden"
      End
   End
   Begin VB.Menu conn_menu 
      Caption         =   "&Verbindung"
      Begin VB.Menu connect_nlg 
         Caption         =   "mit NLG &verbinden"
         Shortcut        =   {F4}
      End
      Begin VB.Menu disconnect_nlg 
         Caption         =   "Verbindung &trennen"
         Shortcut        =   {F5}
      End
   End
   Begin VB.Menu extra1 
      Caption         =   "E&xtras"
      Begin VB.Menu info_nlg 
         Caption         =   "&Geräte  Info"
      End
      Begin VB.Menu Extra_Options 
         Caption         =   "&Optionen"
      End
      Begin VB.Menu show_comment 
         Caption         =   "&Eigenschaften"
      End
   End
   Begin VB.Menu Help 
      Caption         =   "&Hilfe"
      Begin VB.Menu HelpF1 
         Caption         =   "&ChargeStar Hilfe"
         Shortcut        =   {F1}
      End
      Begin VB.Menu HelpContents 
         Caption         =   "&Inhalt"
      End
      Begin VB.Menu blank4 
         Caption         =   "-"
      End
      Begin VB.Menu HelpAbout 
         Caption         =   "In&fo"
      End
   End
End
Attribute VB_Name = "kennlinie"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / Kennlinie                                                               '
'---------------------------------------------------------------------------------------------------------------'
' MODUL BESCHREIBUNG                  / Stellt Fenster und Funktionen zur Verfügung, um eine Kennlinie für die  '
'                                     / NLG 51x Gerätefamilie zu erzeugen und die Begrenzungen einzustellen.    '
'                                     / Laden und Speichern von Kennlinien in einer Datei sowie Laden einer     '
'                                     / Kennlinie von und zum NLG werden unterstützt.                           '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' HISTORY   / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / Okt. '01  / Quang H.    / Rapid Prototype (v.a. Benutzeroberfläche)                               '
' geändert  / Nov. '01  / Beat G.     / Umbau und Erweiterung der Software, Datenhaltung in Datenkapsel,..      '
'           / - Apr'02  / Beat G.     /..neue Funktionen, Speicherung, Fehlerbehandlung etc.                    '
' V1.08     / 26.11.02  / Beat G.     / bg009 geordneter Logout an RS232 wenn Fenster "hart" abgeschossen       '
' V2.0      / April '03 / Beat G.     / mehrsprachige Version (Deutsch / Englisch)                              '
' V2.0      / Mai '03   / Beat G.     / bg018 Datei-Name/Pfad kürzen, falls zu lang für Titelbalken             '
' V2.0      / Juli '03  / Beat G.     / bg021 Temperatur-Einstellungen disablen, falls kein Temp.sensor aktiv   '
' V3.0      / Nov. '04  / Beat G.     / bg024 NLG5-Typ im File nicht mit Default überschreiben                  '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
'                                     /                                                                         '
' MSComm1_OnComm                      / "Interrupt" - Funktion, empfängt und puffert ankommende Zeichen         '
' ConnectNLG                          / Verbindung mit NLG aufbauen (einloggen etc.)                            '
' DisconnectNLG                       / Verbindung mit NLG abbrechen (ausloggen etc.)                           '
' SendReceiveProfile                  / Kennlinie versenden oder empfangen                                      '
' ReceiveReadOnlyInfo                 / "Vorspann" zur Kennlinie (nur lesbar) vom NLG holen                     '
' SendFrame                           / Datenstrom (Frame) byteweise an der seriellen Schnittstelle absetzen    '
' WrapString                          / Datenstrom mit Länge, CRC usw. versandbereit machen                     '
' WaitForReply                        / Antwort von NLG auf versendetes Frame empfangen und auswerten           '
' WaitForSent                         / Warten, bis eine Message vom PC abgesetzt ist                           '
' WaitForEcho                         / Auf Echo-Zeichen warten, für Login                                      '
' WaitSeconds                         / Bestimmte Anzahl Sekunden (z.B. 1.25) warten                            '
' CalculateCRC                        / Checksumme des zu sendenden Datenstroms berechnen                       '
' CheckFrameLengthAndCRC              / Länge und Checksumme eines Frames auf Korrektheit prüfen                '
' FnGetByteFromFrame                  / ein bestimmtes Byte aus einem Frame holen                               '
'                                     /                                                                         '
' PutGuiValToCapsule                  / Im GUI angezeigten Wert in die Kapsel schreiben                         '
' GetGuiValFromCapsule                / Wert für GUI - Anzeige aus der Kapsel holen                             '
' UpdateGUI                           / Refresh der aktuellen Anzeige, aus Datenkapsel                          '
' InitPosPropArrays                   / Arrays für Verknüpfung GUI - Datenkapsel initialisieren                 '
' WriteValToGUI                       / Wert einer Property aus der Kapsel ins entsprechende Control schreiben  '
'                                     /                                                                         '
' Form_Load                           / Kennlinien - Fenster laden und initialisieren                           '
' ***_Click                           / Eingaben von GUI-Controls behandeln, rufen i.d.R. Dienstfunktionen auf  '
'                                     /                                                                         '
' DisplayErrOrMessage                 / Fehler, Warnung usw. anzeigen (allg. Error/Exception - Handler)         '
' GetFileName                         / Datei Dialog (laden/speichern) anzeigen, Name und Pfad eingeben         '
' AskForSaveProfile                   / Abfragen, ob die aktuelle Kennlinie gespeichert werden soll             '
' GetProfileFromFile                  / Kennlinie aus einer bestimmten Datei lesen, in Datenkapsel ablegen      '
' PutProfileToFile                    / Kennlinie aus Datenkapsel holen, in eine bestimmte Datei schreiben      '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'

'
' Anmerkung zum Daten-Abgleich: wenn Daten in der zentralen Datenkapsel verändert werden, so muss dies unmittelbar..
'..eine Neu-Anzeige im GUI zur Folge haben. Faktisch wird jeder Datenabgleich bzw. jedes updaten an der GUI-Anzeige..
'..aufgehängt
'
'
Option Explicit
'
'
'-------------------------------------- Konstanten ------------------------------------------------'
'
'---------------- Verknüpfungen von GUI - Controls mit den Kennlinien - Daten ----
'
' Um die Zuordnung von Control zu Parameter universell und flexibel zu gestalten, wird folgendes Modell verwendet:
' Ein statischer Array enthält an der entsprechenden Indexnummer die entsprechenden Prop_ID's; damit wird in..
'..beide Richtungen (GUI <-> Kapsel) die Zuordnung eindeutig festgelegt. Die entsprechenden Arrays werden in einer..
'..separaten Funktion "InitPosPropArrays" initialisiert
'
'
' relativ privat, gilt im Prinzip nur für dieses Formular
' hier stehen die Index-Nummern der GUI-Controls, die zur entsprechenden Property gehören
' die "allgemeinen Einstellungen", soweit sie Index-adressiert sind (Ausnahme: Temp-Reduktions-Art und Betriebs-Art)
Const GUI_BATT_NENN As Integer = 0    ' Batterie - Nennspannung
Const GUI_TEMP_KOEF As Integer = 1    ' Temperatur - Koeff. Gasung
Const GUI_BEZUG_TMP As Integer = 2    ' Bezugs - Temp Spannungen
Const GUI_MAX_P_NEZ As Integer = 3    ' Max. Netz - Leistung
Const GUI_MAX_I_NEZ As Integer = 4    ' Max. Netz - Strom
Const GUI_DERAT_HYS As Integer = 5    ' Temp. Derating
Const GUI_BEZG_HYST As Integer = 6    ' Temp. Derating Bezugstemp

' die Temperatur-Sensoren
Const GUI_AKT_TMP_1 As Integer = 0    ' aktueller Temperatur-Sensor #1
Const GUI_AKT_TMP_2 As Integer = 1    ' aktueller Temperatur-Sensor #2
Const GUI_AKT_TMP_3 As Integer = 2    ' aktueller Temperatur-Sensor #3

' die Sofort - Abschalt - Kriterien
Const GUI_TMP_UEBER As Integer = 7    ' Über Temp. Batterie
Const GUI_SPG_UEBER As Integer = 8    ' Über Spannung Batterie
Const GUI_ENERG_UEB As Integer = 9    ' Netz-Energie mehr als .. kWh
Const GUI_LADEN_UEB As Integer = 10   ' Batt. Ladung mehr als .. Ah
Const GUI_LADE_ZEIT As Integer = 11   ' Ladezeit länger als .. h
Const GUI_BOOST_MIN As Integer = 12   ' ext. Netzstrom für minimale Booster-Leistung
Const GUI_BOOST_MAX As Integer = 13   ' ext. Netzstrom für maximale Booster-Leistung

' die U-Phase / I-Phase Parameter
Const GUI_I_PH_MAX  As Integer = 1    ' I-Phase max. Ladestrom
Const GUI_U_PH_MAX  As Integer = 2    ' U-Phase max. Ladespg
Const GUI_I_PH_UEB  As Integer = 3    ' I-Phase Überladung %
Const GUI_U_PH_UEB  As Integer = 4    ' U-Phase Überladung %
Const GUI_I_PH_STL  As Integer = 3    ' I-Phase Status LED links
Const GUI_I_PH_STM  As Integer = 2    ' I-Phase Status LED mitte
Const GUI_I_PH_STR  As Integer = 1    ' I-Phase Status LED rechts
Const GUI_U_PH_STL  As Integer = 6    ' U-Phase Status LED links
Const GUI_U_PH_STM  As Integer = 5    ' U-Phase Status LED mitte
Const GUI_U_PH_STR  As Integer = 4    ' U-Phase Status LED rechts
Const GUI_U_PH_AEG  As Integer = 1    ' zurück zu Abschnitt 1 wenn Aus/Ein
Const GUI_U_PH_SG1  As Integer = 2    ' zurück zu Abschnitt 1 bei Signal 1
Const GUI_U_PH_SG2  As Integer = 3    ' zurück zu Abschnitt 1 bei Signal 2
Const GUI_U_PH_S12  As Integer = 4    ' zurück zu Abschnitt 1 bei Signal 1 und 2

' die Vorwärts Abschnittswechsel Kriterien
Const GUI_VW_BED_1  As Integer = 1    ' Vorwärts Bedingung 1 (Typ)
Const GUI_VW_VAL_1  As Integer = 1    ' Vorwärts Bedingung 1 (Wert)
Const GUI_VW_AO_12  As Integer = 1    ' Verknüpfung And/Or Bedingungen 1 und 2
Const GUI_VW_BED_2  As Integer = 2    ' Vorwärts Bedingung 2 (Typ)
Const GUI_VW_VAL_2  As Integer = 2    ' Vorwärts Bedingung 2 (Wert)
Const GUI_VW_AO_23  As Integer = 2    ' Verknüpfung And/Or Bedingungen 2 und 3
Const GUI_VW_BED_3  As Integer = 3    ' Vorwärts Bedingung 3 (Typ)
Const GUI_VW_VAL_3  As Integer = 3    ' Vorwärts Bedingung 3 (Wert)
' bg019 max 5 Kriterien
Const GUI_VW_AO_34  As Integer = 3    ' Verknüpfung And/Or Bedingungen 3 und 4
Const GUI_VW_BED_4  As Integer = 4    ' Vorwärts Bedingung 4 (Typ)
Const GUI_VW_VAL_4  As Integer = 4    ' Vorwärts Bedingung 4 (Wert)
Const GUI_VW_AO_45  As Integer = 4    ' Verknüpfung And/Or Bedingungen 4 und 5
Const GUI_VW_BED_5  As Integer = 5    ' Vorwärts Bedingung 5 (Typ)
Const GUI_VW_VAL_5  As Integer = 5    ' Vorwärts Bedingung 5 (Wert)


'---------------- sonschtige GUI Konstanten ----
Const GUI_DEF_FILE_EXT_IDX  As Integer = 2  ' Index in der Endungsliste des Datei-Dialogs ist 2, für *.pfl

'----------------------------------------- Ende Konstanten -----------------------------------------'



'----------------------------------------- modulglobale Variablen ----------------------------------'
'


'------------------ die Arrays (oder Einzel-Var), welche Controls zu Parameter-ID's zuordnen ---------

Dim lGuiParArrayAllg(13)  As Long     ' die indizierten Par der allgemeinen Einstellungen (Edit-Böxli)
Dim lGuiParAktTempSens(3) As Long     ' die Anzahl aktiver Temperatur-Sensoren
Dim lGuiParRedArt         As Long     ' die Temperatur Reduktions-Art
Dim lGuiParBetrArt        As Long     ' die Betriebsart (Standard/Master/Booster) des NLG
' N.B. die Abschnitts-ComboBox gehört nicht dazu, sie enthält nur die Information, welcher Abschnitt gerade angezeigt wird
Dim lGuiGenAbEdBox(4)     As Long     ' Edit Boxen des "Generell" Feldes (Strom, Spannung, Ueberladung)
Dim lGuiStatusLeds(6)     As Long     ' diese 6 Felder sind 2 Par; 1 Par hat 3 Leds à 3 Zustände (0,1,2)
Dim lGuiRetCheckBox(4)    As Long     ' 4 Checkboxen (0,1), je durch einen eigenen Par repräsentiert
' N.B. die WechselBedingungs-ComboBox gehört auch nicht dazu, sie enthält eine temporäre Information
Dim lGuiTypRueck(5)       As Long     ' Typ der Rückwärts - Bedingungen
Dim lGuiWertRueck(5)      As Long     ' Werte der Rückwärts - Bedingungen
Dim lGuiAndOrRueck(4)     As Long     ' Verknüpfungen zwischen Rückwärts - Bedingungen
Dim lGuiTypVor(5)         As Long     ' Typ der Vorwärts - Bedingungen
Dim lGuiWertVor(5)        As Long     ' Werte der Vorwärts - Bedingungen
Dim lGuiAndOrVor(4)       As Long     ' Verknüpfungen zwischen Vorwärts - Bedingungen



' -------------- Variablen für Kommunikation ---------------------

' Mangels sinnvoller Alternativen wurde die Datenhaltung des Sende - Streams mit einem globalen Array
' yOutStream realisiert; dito CRC. Und: da das Kommunikationsobjekt am entsprechenden Formular (z.B.
' Kennlinien - Programm) hängt, muss die die Empfangsfunktion "MSComm1_OnComm" auch da platziert sein.

Dim yOutStream(COM_MAX_LEN) As Byte     ' globaler Array für den Out - Stream
Dim yHiCRC As Byte, yLoCRC As Byte      ' globale Variablen für CRC Hi und Lo Byte

' "Schnittstelle" zur Empfangs - Funktion (Telefönli-Objekt, Comm1)
Dim strInBuffer As String               ' String variabler Länge, hier kommen die empfangenen Daten rein
Dim strRecFrameBuf As String            ' empfangenes, komplettes Frame zwischenspeichern zum Verarbeiten

Dim ret As Integer                      ' nur für Debug

'Dim bLockLogin As Boolean               ' Um Verklemmungen der Kommunikation zu vermeiden, wird nach einem..
'                                        '..Logout das Login für eine Weile gesperrt


' -------------- sonstige modulglobale Variablen ------------------------

Dim bInvalidCWNumEd1      As Boolean    ' besagt, dass die Werte des Controls CWNumEdit1 momentan ungültig sind
Dim bGUIToCapsule         As Boolean    ' besagt, dass Werte vom GUI in die Datenkapsel geschrieben werden dürfen
Dim bBlinkState           As Boolean    ' enthält den aktuellen Status der Blink-Items
Dim sCurrentHeight        As Single     ' enthält die momentane maximale Fensterhöhe
Dim iBackUpNLGType        As Integer    ' NLG5-Typ, von der Datei gelesen
'
'
'
'------------------------------ Ende Variablen-Feld---------------------------------------------
'






Private Sub Extra_Options_Click()
  
  FrmOptions.Show vbModal

End Sub

'///////////////////////////////////////////////////////////////////////////////////////////////////////////////
'//                                                                                                           //
'// Die Kommunikationsfunktionen, müssen vorerst im Hauptmodul liegen (ev. später auslagern)                  //
'//                                                                                                           //
'///////////////////////////////////////////////////////////////////////////////////////////////////////////////







'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Empfängt ankommende Zeichen, schiebt diese in den (temporären) Buffer und erhöht dessen Zähler um 1; wenn..   '
'.. gültiges Frame erkannt, d.h. Länge und Overhead stimmen, wird die Auswertung aufgerufen                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Static Sub MSComm1_OnComm()

Static bBufOverflow   As Boolean                      ' zeigt an, wenn ein Überlauf im Empfangspuffer vorgekommen ist

  'Debug.Print MSComm1.CommEvent                      ' ddd "sende das Kommunikatons - Ereignis ans Direktfenster"
    
  If MSComm1.CommEvent = comEvReceive Then            ' etwas empfangen
    
    g_strEmpfString = ""
    g_strEmpfString = MSComm1.Input                   ' Eingangspuffer holen / löschen
      
    strInBuffer = strInBuffer & g_strEmpfString       ' empfangenen String an Puffer anhängen
    
    ' Wenn Länge des Strings kleiner gleich max. Länge, ist's ok; sonst: Fehler
    If Len(strInBuffer) <= COM_MAX_LEN Then
      If (Len(strInBuffer) > COM_POS_VAL_START) Then  ' ab dieser Länge könnte es ein korrektes Frame sein
        If Asc(Right(strInBuffer, 1)) = COM_ETX Then  ' auf "ETX" (Ascii 3) triggern, also Ende Frame
          ' Wenn Länge des Strings wie angekündigt, dann..
          If Asc(Mid(strInBuffer, COM_POS_LENGTH + 1, 1)) = Len(strInBuffer) Then
            strRecFrameBuf = strInBuffer              ' empfangenes, komplettes Frame puffern (noch nicht auf Korrektheit geprüft)
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
    
End Sub                                 ' Ende MSComm1_OnComm




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Verbindung zum NLG aufbauen bzw. abbrechen, login, steuerbefehle absetzen etc. Das eigentliche Öffnen des..   '
'..Comm - Ports muss beim Objekt selber geschehen, also im Kennlinienprogramm                                   '
'                                                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function ConnectNLG() As Integer
  
  Dim lVal      As Long                                   ' Rückgabewert der Antwort
  Dim iRetry    As Integer                                ' Sende - Zähler
  Dim yHi       As Byte                                   ' Hi - Byte der Property
  Dim yLo       As Byte                                   ' Lo - Byte der Property
  Dim strAux    As String                                 ' Hilfs - String
  Dim bErr      As Boolean                                ' Fehler in der Kommunikation
  Dim bDone     As Boolean                                ' Flag für "Funktion beendet"
  Dim bSuccess  As Boolean                              ' Verbindungsaufnahme erfolgreich
  Dim iWhatNow  As Integer                              ' Was tun?
  Dim fTimeStamp As Single                              ' Zeitstempel für's Warten
  Dim bLockLogin As Boolean                             ' Login-Verklemmung passiert
  Dim iLoop     As Integer                              ' Schleifenvariable
  Dim yAuxStream(COM_MAX_LEN) As Byte

On Error GoTo LBL_COM_PORT_ERROR    ' Sch..VB ist nicht in der Lage zu erkennen, ob ein *fremdes* Programm den..
                                    '..Comm-Port1 schon benutzt, sondern nur ob es ihn *selbst* schon geöffnet hat.
                                    ' Folge: wenn der Comm-Port1 belegt ist, gibt's einen Laufzeitfehler -> abfangen
                                    
bLockLogin = False                  ' Login not locked yet

LBL_START_AGAIN:                                        ' Sprungmarke für "erneut verbinden versuchen"

  If MSComm1.PortOpen = False Then                      ' serielle Schnittstelle öffnen
    
    MSComm1.CommPort = 1
    MSComm1.Settings = "19200,n,8,1"
    MSComm1.InputLen = 1
    MSComm1.RThreshold = 1
    MSComm1.SThreshold = 1
    MSComm1.PortOpen = True
  Else                                ' serielle Schnittstelle schon offen
    If (SWITCH_DEBUG = True) Then
      MsgBox ("Bei ConnectNLG besteht schon eine Verbindung")
    End If
  End If


  
  If (bLockLogin = True) Then           ' bg025 Verklemmungen verhindern, indem ein Quit + Enter..
                                        '.."qq+LF" in mässigem Tempo abgesetzt werden
    For iRetry = 1 To 3
    
      WaitSeconds (LOG_TIMEOUT)
      
      yOutStream(COM_MAX_LEN) = 1
      
      yOutStream(0) = 113        'q
      Call SendFrame                                      ' erstes Zeichen senden
      WaitForEcho (Chr(yOutStream(0)))
      
      yOutStream(0) = 113        'q
      Call SendFrame                                      ' erstes Zeichen senden
      WaitForEcho (Chr(yOutStream(0)))


      yOutStream(0) = 10        'LF
      Call SendFrame                                      ' erstes Zeichen senden
      WaitForEcho (Chr(yOutStream(0)))
    
    Next iRetry
    
  End If
  
  
    yOutStream(COM_MAX_LEN) = 1
      
      
    yOutStream(0) = 113        'q
    Call SendFrame                                      ' erstes Zeichen senden
    WaitForEcho (Chr(yOutStream(0)))
      
    yOutStream(0) = 113        'q
    Call SendFrame                                      ' erstes Zeichen senden
    WaitForEcho (Chr(yOutStream(0)))


    yOutStream(0) = 10        'LF
    Call SendFrame                                      ' erstes Zeichen senden
    WaitForEcho (Chr(yOutStream(0)))
                                                        ' Login - String erstellen
    
    yAuxStream(0) = 112       'p
    yAuxStream(1) = 114       'r
    yAuxStream(2) = 111       'o
    yAuxStream(3) = 102       'f
    yAuxStream(4) = 105       'i
    yAuxStream(5) = 108       'l
    yAuxStream(6) = 101       'e
    yAuxStream(7) = 13        'cr
    yAuxStream(8) = 10        'lf
    
    WaitSeconds (5 * LOGIN_WAIT)                        ' warten, bis "com1 login: " fertig und NLG wieder bereit
    
    For iLoop = 0 To 8                                  ' bg025 login string nicht "am Stück" senden
      yOutStream(0) = yAuxStream(iLoop)
      yOutStream(COM_MAX_LEN) = 1
      Call SendFrame                                    ' Login - String zeichenweise senden
      WaitForEcho (Chr(yOutStream(0)))
    Next iLoop                                          ' end bg025
    
    bErr = True                                         ' Initialisierungen
    iRetry = 0                                          ' Sende - Zähler initialisieren
    bSuccess = True                                     ' Verbindung erfolgreich
    
                                                        ' prop "Typ" abfragen:
    yHi = FnHiByteOfLong(INF_NLG_TYP_51X)               ' Hi - NLG-Typ
    yLo = FnLoByteOfLong(INF_NLG_TYP_51X)               ' Lo - NLG-Typ
    Call WrapString(OP_GET, yHi, yLo, "")               ' Abfrage, keine Daten mitsenden
  
  
  
  
    While ((bErr = True) And (bSuccess))                ' ggf. mehrere Versuche
      Call SendFrame                                    ' Frame senden
      Call WaitForReply(OP_GET, yHi, yLo, lVal, bErr, bDone)    ' Antwort von NLG abfragen / abwarten
      iRetry = iRetry + 1                                       ' Anz. Sendeversuche inkrementieren
      
      If ((bErr = True) And (iRetry >= LOGIN_RETRY)) Then       ' alle Versuche erfolglos
        iWhatNow = DisplayErrOrMessage(CONNECT_ERROR, "")       ' nochmals versuchen oder abbrechen?
        If (iWhatNow = vbRetry) Then
          MSComm1.PortOpen = False                      ' Port wieder schliessen, versuch's nochmals von vorne
          bLockLogin = True
          GoTo LBL_START_AGAIN                          ' sorry...aber das GoTo ist in diesem Fall vernünftig
        Else
          bSuccess = False                              ' vergiss es
          MSComm1.PortOpen = False                      ' Port wieder schliessen
        End If
      End If
    
    Wend
  
    If (bSuccess = True) Then                           ' Verbindung zustande gekommen..
      Profile_to.Enabled = True                         '..also Kennlinie zum Gerät schicken möglich..
      Profile_from.Enabled = True                       '..und Kennlinie vom Gerät holen möglich..
      connect_nlg.Enabled = False                       '..und erneut verbinden nicht möglich..
      disconnect_nlg.Enabled = True                     '..und Verbindung abbrechen möglich und ..
      info_nlg.Enabled = True                           '..Info abfragen nur online möglich und..
      status_fenster(1).Visible = True                  ' zeige "connected" an
      status_fenster(0).Visible = False
      g_bConnectionEstablished = True                   ' eingeloggt
    End If
  

  
  Exit Function                               ' nix Fehler, also aussteigen


LBL_COM_PORT_ERROR:

  Call DisplayErrOrMessage(Err.Number, "")
  On Error GoTo 0                                       ' VB-Fehlerbehandlung deaktivieren bzw. verlassen

End Function                                          ' Ende ConnectNLG




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Verbindung zum NLG abbrechen: Logout, ggf. Steuerbefehle absetzen                                             '
'                                                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function DisconnectNLG() As Integer
  
  Dim bErr    As Boolean                                ' Fehler in der Kommunikation
  Dim bDone   As Boolean                                ' Flag für "Funktion beendet"
  Dim yHi     As Byte                                   ' Hi - Byte der Property
  Dim yLo     As Byte                                   ' Lo - Byte der Property
  Dim iErr    As Integer                                ' Fehler - zähler
  Dim lVal    As Long                                   ' Rückgabewert der Antwort
  Dim fTimeStamp As Single                              ' Zeitstempel für's Warten
  



On Error GoTo LBL_DISCONNECT_ERROR    ' Sch..VB ist nicht in der Lage zu erkennen, ob ein *fremdes* Programm den..
                                    '..Comm-Port1 schon benutzt, sondern nur ob es ihn *selbst* schon geöffnet hat.
                                    ' Folge: wenn der Comm-Port1 belegt ist, gibt's einen Laufzeitfehler -> abfangen


  If MSComm1.PortOpen = True Then                   ' serielle Kommunikation offen?
    
    yHi = FnHiByteOfLong(DCC_EOT_LOG_OUT)           ' ID von Logout laden
    yLo = FnLoByteOfLong(DCC_EOT_LOG_OUT)           ' ID von Logout laden
    
    iErr = 0                                        ' Fehler - Zähler reseten
    
    Call WrapString(OP_SET, yHi, yLo, Chr(COM_LOG_OUT))  ' Logout Befehl erzeugen
      
    Do                                              ' versuche, das Frame abzusetzen
      Call SendFrame
      Call WaitForReply(OP_SET, yHi, yLo, lVal, bErr, bDone) ' Antwort von NLG abfragen / abwarten
      If (bErr = True) Then                         ' Fehler-Zähler++
        iErr = iErr + 1
      End If
    Loop Until ((bErr = False) Or (iErr > LOGIN_RETRY)) ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
    
    If (iErr > LOGIN_RETRY) Then                    ' Logout fehlgeschlagen
      
      If (SWITCH_DEBUG = True) Then
        MsgBox ("geordneter Logout nicht möglich")
      End If
    
    Else                                            ' Logout erfolgreich
      
      fTimeStamp = Timer                            ' Zeitstempel = aktuelle Systemzeit in Sekunden
      Do                                            ' "Halb-Busy-Wait" zum ausloggen des NLG
        ret = DoEvents()                            ' Windows Ereignisse ausführen ("kooperatives Multitasking")
      Loop Until ((Timer - fTimeStamp) > LOG_TIMEOUT) ' Timeout prüfen
      
      MSComm1.PortOpen = False                      ' serielle Schnittstelle schliessen..
      Profile_to.Enabled = False                    '..also Kennlinie zum Gerät schicken nicht möglich..
      Profile_from.Enabled = False                  '..und Kennlinie vom Gerät holen nicht möglich..
      connect_nlg.Enabled = True                    '..wieder verbinden möglich..
      disconnect_nlg.Enabled = False                '..erneutes Logout nicht möglich..
      info_nlg.Enabled = False                      '..Info abfragen offline nicht möglich..
      status_fenster(1).Visible = False             '..zeige "not connected" an
      status_fenster(0).Visible = True
      NLGTyp.Text = "- - -"                         ' Display Anzeige "nix"
      NLGTyp.Refresh
      g_bConnectionEstablished = False              ' ausgeloggt
    End If
  
  Else                                ' serielle Schnittstelle schon geschlossen
    If (SWITCH_DEBUG = True) Then
      MsgBox ("Serielle Schnittstelle schon geschlossen")
    End If
  End If
  

  Exit Function                               ' nix Fehler, also aussteigen

LBL_DISCONNECT_ERROR:

  Call DisplayErrOrMessage(Err.Number, "")
  On Error GoTo 0                                       ' VB-Fehlerbehandlung deaktivieren bzw. verlassen
  
  
End Function                                          ' Ende DisconnectNLG





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden oder Empfangen der ganzen Kennlinie. Will heissen, vorgegebenen Array mit allen  Parametern ..         '
'..durchzählen (Datenkapsel füllen / auslesen), einen Befehl senden, auf Quittierung warten, ggf. Befehl..      '
'..wiederholen. Fortschritts - Balken in Modalem Dialog anzeigen.                                               '
' Zusätzlich für Senden: "StopCharge" / "StartCharge" Sequenz senden (bzw. enable/disable charge)               '
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
  Dim bDone   As Boolean                              ' Flag für "Funktion beendet"
  Dim iWhatNow  As Integer                            ' Was tun bei Fehler?
  
  
  If MSComm1.PortOpen = True Then                     ' serielle Kommunikation offen?
    
    iCount = 0                                        ' Zähler für übertragene Prop's initialisieren
    g_bMsgStopTransmitData = False                    ' Abbruch - Flag für Uebertragung löschen
    
    yHiPrev = NOT_PAR                                 ' Am Anfang ist der "vorherige Par" der "NichtParameter" an Stelle 0
    yLoPrev = NOT_PAR                                 '
      
    UpDownLoad.Show vbModeless                        ' Böxli mit Fortschrittsbalken anzeigen
    UpDownLoad.ProgressBar1.Min = 0                   ' Minimal - Wert der Übertragung
    
    If (bSet = True) Then                             ' Kennlinie zum NLG senden
      UpDownLoad.Caption = LoadResString(STR_ID_RTXBOX_SEND + g_lLocaleOffset)
      UpDownLoad.ProgressBar1.Max = NUM_USED_PAR - NUM_USED_RDONLY  ' Max. Anzahl zu sendender Par
    Else                                              ' Kennlinie vom NLG holen
      UpDownLoad.Caption = LoadResString(STR_ID_RTXBOX_RECEIVE + g_lLocaleOffset)
      UpDownLoad.ProgressBar1.Max = NUM_USED_PAR      ' Max. Anzahl zu empfangender Par
    End If
      
      
                                ' ------------- bei senden : Ladevorgang sperren/unterbrechen --------------
    If (bSet = True) Then                               ' zum Senden der Kennlinie Laden verhindern
      strTmp = Chr(COM_STOP_CHG)                        ' Byte "Laden verboten" als String
      yHi = FnHiByteOfLong(DCC_CHARGE_START_STOP)
      yLo = FnLoByteOfLong(DCC_CHARGE_START_STOP)
      Call WrapString(OP_SET, yHi, yLo, strTmp)         ' Frame erzeugen
      Do                                                ' versuche, das Frame abzusetzen
        Call SendFrame
        Call WaitForReply(OP_SET, yHi, yLo, lVal, bErr, bDone)     ' Antwort von NLG abfragen / abwarten
        If (bErr = True) Then                           ' Fehler-Zähler
          iErr = iErr + 1
        End If
      Loop Until ((bErr = False) Or (iErr > ERR_RETRY)) ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
      
      If (iErr > ERR_RETRY) Then                        ' Max. Error Retries erreicht?
        Call DisplayErrOrMessage(CHARGE_STOP_ERROR, "") ' abbrechen
        GoTo LBL_FAILED                                 ' nix Erfolg, Funktion verlassen
      End If
    End If
    
      
                                ' ------------- alle Parameter durchzählen --------------
    While ((GetNextPar(yHiPrev, yLoPrev, yHi, yLo) = True) And (g_bMsgStopTransmitData = False))
      
      iErr = 0                                          ' Fehler - Zähler reseten
      
      lVal = GetDataFromCapsule(yHi, yLo)               ' Wert des aktuellen Parameters holen
      
      strTmp = Chr(FnHiByteOfLong(lVal))                ' High Byte als String
      strTmp = strTmp & Chr(FnLoByteOfLong(lVal))       ' Low Byte als String anhängen
          
      If (bSet = True) Then                   '--------- Werte Setzen/Kennlinie senden---------- ?
          
        If (GetStatusOfPar(yHi, yLo) = True) Then       ' Darf der Wert des Parameters überschrieben werden?
      
          iCount = iCount + 1                           ' 1 weitere Property übertragen
          
          Call WrapString(OP_SET, yHi, yLo, strTmp)     ' Frame erzeugen
          
LBL_RETRY_SEND:     ' Sprungmarke für "versuche gleiches Frame nochmals zu senden"
          Do                                            ' versuche, ein Frame abzusetzen
            Call SendFrame
            Call WaitForReply(OP_SET, yHi, yLo, lVal, bErr, bDone)     ' Antwort von NLG abfragen / abwarten
            If (bErr = True) Then                       ' Fehler-Zähler
              iErr = iErr + 1
            End If
          Loop Until ((bErr = False) Or (iErr > ERR_RETRY))     ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
          
          If (iErr > ERR_RETRY) Then                            ' Max. Error Retries erreicht?
            iWhatNow = DisplayErrOrMessage(FRAME_SEND_ERROR, GetTextOfProperty(yHi, yLo)) ' neu versuchen, ignorieren oder abbrechen?
            If (iWhatNow = vbRetry) Then                ' neu versuchen
              iErr = 0
              GoTo LBL_RETRY_SEND                       ' sorry...aber sinnvoll
            Else
              If (iWhatNow = vbAbort) Then
                g_bMsgStopTransmitData = True           ' Abbruch
              End If
            End If                                      ' wenn nicht "retry" oder "cancel", dann "ignore", also nächste Property
          End If
        End If                                          '
          
      Else                                  '----------- nicht setzen/senden, sondern abfragen --------
            
        iCount = iCount + 1                 ' 1 weitere Property übertragen
        strTmp = ""                                 ' beim Abfragen werden keine Daten mitgesendet
        Call WrapString(OP_GET, yHi, yLo, strTmp)
        
LBL_RETRY_RECEIVE:     ' Sprungmarke für "versuche gleiches Frame nochmals zu empfangen"
        Do                                          ' versuche, ein Frame abzusetzen
          Call SendFrame
          Call WaitForReply(OP_GET, yHi, yLo, lVal, bErr, bDone)   ' Antwort von NLG abfragen / abwarten
          If (bErr = True) Then                     ' Fehler-Zähler
            iErr = iErr + 1
          End If
        Loop Until ((bErr = False) Or (iErr > ERR_RETRY))     ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
        If (iErr > ERR_RETRY) Then                            ' erfolglos, also Fehlermeldung anzeigen
          iWhatNow = DisplayErrOrMessage(FRAME_RECEIVE_ERROR, GetTextOfProperty(yHi, yLo))
          If (iWhatNow = vbRetry) Then                ' neu versuchen
            iErr = 0
            GoTo LBL_RETRY_RECEIVE                    ' sorry...aber sinnvoll
          Else
            If (iWhatNow = vbAbort) Then
              g_bMsgStopTransmitData = True           ' Abbruch
            End If
          End If                                      ' wenn nicht "retry" oder "cancel", dann "ignore", also nächste Property
        End If
      End If                                '----------- Ende setzen / abfragen -------------------------
      
      yHiPrev = yHi
      yLoPrev = yLo
      
                                          ' aktuellen Wert anzeigen, sicherstellen dass er < 100% ist (sonst gibt's 'n Fehler)
      UpDownLoad.ProgressBar1.Value = FnMin(iCount, UpDownLoad.ProgressBar1.Max)
    
    Wend                                  ' ------------- Ende Parameter durchzählen --------------
  
                                ' ------------- Ladevorgang zulassen/neu starten --------------
    If (bSet = True) Then                               ' Senden der Kennlinie fertig, Laden wieder erlaubt
      strTmp = Chr(COM_START_CHG)                       ' Byte "Ladung erlaubt" als String
      yHi = FnHiByteOfLong(DCC_CHARGE_START_STOP)
      yLo = FnLoByteOfLong(DCC_CHARGE_START_STOP)
      Call WrapString(OP_SET, yHi, yLo, strTmp)         ' Frame erzeugen
      Do                                                ' versuche, das Frame abzusetzen
        Call SendFrame
        Call WaitForReply(OP_SET, yHi, yLo, lVal, bErr, bDone)     ' Antwort von NLG abfragen / abwarten
        If (bErr = True) Then                           ' Fehler-Zähler
          iErr = iErr + 1
        End If
      Loop Until ((bErr = False) Or (iErr > ERR_RETRY)) ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
      
      If (iErr > ERR_RETRY) Then                        ' Max. Error Retries erreicht?
        Call DisplayErrOrMessage(CHARGE_START_ERROR, "") ' abbrechen
        GoTo LBL_FAILED                                 ' nix Erfolg, Funktion verlassen
      End If
    End If
    

LBL_FAILED:       'Sprungmarke für fehlgeschlagene Kommunikation
    Unload UpDownLoad                     ' Fortschritts - Fensterli schliessen


  
  Else                                                ' Verbindung nicht offen, also warnen
    If (SWITCH_DEBUG = True) Then
      MsgBox ("Bei SendReceiveProfile besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If

End Sub                                 ' --- Ende SendReceiveProfile




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Empfangen der Geräte - Info, also der nicht beschreibbare Vorspann vor der eigentlichen Kennlinie             '
'                                                                                                               '
' bOk            : Lesen von Gerät erfolgreich / nicht erfolgreich                                              '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub ReceiveReadOnlyInfo(bOk As Boolean)
  
  Dim bErr    As Boolean                              ' lokaler Fehler
  Dim yHi As Byte, yLo As Byte                        ' Hi und Lo Byte der Prop_ID des aktuellen Parameters
  Dim yHiPrev As Byte, yLoPrev As Byte                ' Prop_ID's des vorherigen Par's
  Dim iCount  As Integer                              ' lokale Zählvariable
  Dim lVal As Long                                    ' temporäre Variable für Wert des Parameters
  Dim strTmp  As String                               ' temporärer String
  Dim iErr  As Integer                                ' Fehler - Zähler, um 'Hänger' zu detektieren
  Dim bDone   As Boolean                              ' Flag für "Funktion beendet"
  
  
  If MSComm1.PortOpen = True Then                     ' serielle Kommunikation offen?
    
    yHiPrev = NOT_PAR                                 ' Am Anfang ist der "vorherige Par" der "NichtParameter" an Stelle 0
    yLoPrev = NOT_PAR                                 ' der "NichtParameter" wird als Ursprung benutzt und nie ausgewertet
    yHi = NOT_PAR
    yLo = NOT_PAR
    
                                ' ------------- alle Parameter durchzählen, die existieren und NUR LESBAR sind --------------
    While ((GetNextPar(yHiPrev, yLoPrev, yHi, yLo) = True) And (GetStatusOfPar(yHi, yLo) = False))
      
      bOk = False                                     ' Funktion auf "erfolglos" initialisieren
      iErr = 0                                        ' Fehler - Zähler reseten
      
      lVal = GetDataFromCapsule(yHi, yLo)             ' Wert des aktuellen Parameters holen
      
      Call WrapString(OP_GET, yHi, yLo, "")           ' nur Abfrage, keine Daten mitsenden
        
      Do                                              ' versuche, ein Frame abzusetzen
        Call SendFrame
        Call WaitForReply(OP_GET, yHi, yLo, lVal, bErr, bDone) ' Antwort von NLG abfragen / abwarten
        If (bErr = True) Then                         ' Fehler-Zähler
          iErr = iErr + 1
        Else
          Call PutDataToCapsule(yHi, yLo, lVal)       ' wenn ok, in aktuellen Datensatz schreiben
          bOk = True                                  ' Funktion bis hierher erfolgreich
        End If
      Loop Until ((bErr = False) Or (iErr > ERR_RETRY)) ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
      
      If (iErr > ERR_RETRY) Then
        If (SWITCH_DEBUG = True) Then
          MsgBox ("Geräte - Info lesen nicht möglich")
        End If
      End If
               
      
      yHiPrev = yHi
      yLoPrev = yLo
    Wend                                  ' ------------- Ende Parameter durchzählen --------------
  
  Else                                                '  Verbindung nicht offen, also warnen
    If (SWITCH_DEBUG = True) Then
      MsgBox ("Bei ReceiveReadOnlyInfo noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If

End Sub                                 ' --- Ende ReceiveReadOnlyInfo





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden des fixfertigen Frames, d.h den in yOutStream enthaltenen Datenstrom Byte für Byte in den Output- ..   '
'..Puffer der RS232 schieben, "kooperativ" warten bis sie abgesetzt sind                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub SendFrame()
  
  Dim iCount As Integer                               ' lokale Zählvariable
  
    
  If MSComm1.PortOpen = True Then                     ' serielle Kommunikation offen?
    
    For iCount = 0 To (yOutStream(COM_MAX_LEN) - 1)   ' ganzes Frame in den OutputBuffer
      MSComm1.Output = Chr(yOutStream(iCount))
    Next iCount
    
    Call WaitForSent                                  ' warten, bis Frame abgesetzt oder Timeout
  
  Else                                                ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Bei SendFrame besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If

End Sub                                 ' Ende SendFrame




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
  
  
  yOutStream(4) = iStringLen + COM_OVERHEAD     ' Länge des Frames inkl. Overhead ermitteln, an 4.Stelle schreiben
  
  For iCount = 1 To iStringLen  ' Werte-String Byte für Byte in den Out - Stream schreiben, an der richtigen Stelle
    yOutStream(COM_POS_VAL_START - 1 + iCount) = Asc(Mid(strInString, iCount, 1))
  Next iCount
  
  ' Checksumme (CRC) über alle Bytes bis hierhin ermitteln und schreiben (im Moment nur Quersumme)
  Call CalculateCRC(iStringLen)                 ' Checksumme ermitteln
  
  yOutStream(COM_POS_VAL_START + iStringLen) = yHiCRC               ' High - Byte der Checksumme an drittletzter Stelle
  yOutStream(COM_POS_VAL_START + 1 + iStringLen) = yLoCRC           ' Low - Byte der Checksumme an zweitletzter Stelle
  yOutStream(COM_POS_VAL_START + 2 + iStringLen) = COM_ETX          ' End - Zeichen für Kommunikation

End Sub                                 ' Ende WrapString




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'                                                                                                               '
' Warten auf passende Antwort zum ausgesendeten Befehl; wenn korrekte Antwort, dann allfälliges Resultat..      '
'..weitergeben, sonst Fehler melden                                                                             '
'                                                                                                               '
' yOpID           : Operation, die auf diese Geräte - Eigenschaft angewendet wird: setzen, lesen, usw.          '
' yPropIDHi/Lo    : Geräte - Eigenschaft: Gerätetyp, Nennspannung der Batterie, Wechsel-Bedingung usw.          '
' lValue          : allfällig zurückgegebener Wert der Eigenschaft                                              '
' bError          : Flag, welches meldet, ob ein Fehler auftrat (True) oder korrekte Antwort kam (False)        '
' bDone           : allfällige Wartezeit ist abgelaufen, es kann weitergehn                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WaitForReply(yOpID As Byte, yPropIDHi As Byte, yPropIDLo As Byte, lValue As Long, bError As Boolean, bDone As Boolean)
  
  Dim iCount As Integer                             ' lokale Zählvariable
  Dim yAuxByte() As Byte                            ' Zwischenspeicher für Daten
  Dim fTimeStamp As Single                          ' Zeitstempel für's Warten
  Dim bTimeOut As Boolean                           ' Flag für Timeout
  
  
  fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
  bError = True                                     ' der wird nur False gesetzt, wenn die Antwort 100%-ig richtig war
  bTimeOut = False                                  ' muss man wohl jedes mal initialisieren
  bDone = False                                     ' Funktion noch nicht ausgeführt
  
  If (MSComm1.PortOpen = True) Then                 ' serielle Kommunikation offen?
    
    Do                                              ' "Halb-Busy-Wait" bis Message da
      If ((Timer - fTimeStamp) > COM_TIMEOUT) Then
        bTimeOut = True                             ' keine Antwort innerhalb nützlicher Frist erhalten
      End If
      ret = DoEvents()                              ' Windows Ereignisse ausführen ("kooperatives Multitasking")
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
                    Case COM_LOG_OUT                                                        ' Logout
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
                Call PutDataToCapsule(yPropIDHi, yPropIDLo, lValue)  '                     ' empfangene Daten in die Kapsel schreiben
                bError = False                                                             ' ok
              End If
            End If
            
          Case Else                                                                       ' Alles Übrige
            If SWITCH_DEBUG = True Then
              MsgBox ("unbekannte OP_ID bei WaitForReply")
            End If
        
        End Select
      
      Else
        bError = True                               ' Frame nicht ok
      End If                                        ' Ende Frame prüfen
    
    Else                                            ' keine rechtzeitige Antwort eingetroffen
      If SWITCH_DEBUG = True Then
        MsgBox ("Timeout bei Warten auf Antwort")
      End If
    End If                                          ' Ende rechtzeitige Antwort eingetroffen
    
  If (bError = True) Then
    iCount = 3
  End If

  g_bMsgReceived = False                            ' empfangene Msg verarbeitet, also Msg löschen
  
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Bei WaitForReply besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

bDone = True                                        ' Funktion beendet

End Sub                             ' Ende WaitForReply




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
  
  
  fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
  bTimeOut = False                                  ' muss man wohl jedes mal initialisieren
  
  If (MSComm1.PortOpen = True) Then                 ' serielle Kommunikation offen?
    
    Do                                              ' "Halb-Busy-Wait" bis Message abgesetzt
      If ((Timer - fTimeStamp) > COM_TIMEOUT) Then
        bTimeOut = True                             ' keine Antwort innerhalb nützlicher Frist erhalten
      End If
      ret = DoEvents()                              ' Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until ((MSComm1.OutBufferCount = BUF_EMPTY) Or (bTimeOut = True))  ' Message abgesetzt oder Timeout erreicht, abbrechen
    
    If (bTimeOut = True) Then                       ' Message konnte nicht in vernünftiger Zeit abgesetzt werden
      If SWITCH_DEBUG = True Then
        MsgBox ("Timeout beim Absetzen eines Frames")
      End If
    End If                                          ' Ende rechtzeitige Antwort eingetroffen
 
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Bei WaitForSent besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

End Sub                                             ' Ende WaitForSent




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
    
    If (bTimeOut = True) Then                       ' Message konnte nicht in vernünftiger Zeit abgesetzt werden
      If SWITCH_DEBUG = True Then
'        MsgBox ("Kein Echo erhalten")
      End If
    End If                                          ' Ende rechtzeitige Antwort eingetroffen
 
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
'      MsgBox ("Bei WaitForEcho besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

End Sub                                             ' Ende WaitForEcho




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Warten bis eine Anzahl Sekunden verstrichen ist, Windös weiterarbeiten lassen.                                '
'                                                                                                               '
' fTimeSeconds:         zu wartende Sekunden (z.B. 1.75, 0.02 etc.)                                             '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WaitSeconds(fTimeSeconds As Integer)
  
  Dim fTimeStamp As Single                          ' Zeitstempel für's Warten
  Dim bTimeOut As Boolean                           ' Flag für Timeout
  
  
  fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
  bTimeOut = False                                  ' muss man wohl jedes mal initialisieren
  
  
  Do                                                ' "Halb-Busy-Wait" bis Message abgesetzt
    If ((Timer - fTimeStamp) > fTimeSeconds) Then
      bTimeOut = True                               ' keine Antwort innerhalb nützlicher Frist erhalten
    End If
    ret = DoEvents()                                ' Windows Ereignisse ausführen ("kooperatives Multitasking")
  Loop Until (bTimeOut = True)                      ' Echo da oder Timeout, weiter
    
End Sub                                             ' Ende WaitSeconds




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

  yHiCRC = FnHiByteOfLong(lLocCRC)        ' High Byte der Quersumme ermitteln
  yLoCRC = FnLoByteOfLong(lLocCRC)        ' Low Byte der Quersumme ermitteln

End Sub                               ' Ende CalculateCRC



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Prüft ein übergebenes Frame auf korrekte Länge und korrekte Checksumme; Funktion z.T. analog "CalculateCRC"   '
'                                                                                                               '
' strFrame   : zu prüfendes Empfangs-Frame                                                                      '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function CheckFrameLengthAndCRC(strFrame As String) As Boolean

Dim iCount As Integer                       ' Zähler
Dim lCRC As Long                            ' CRC


If (Len(strFrame) > COM_OVERHEAD) Then      ' "sinnvolle" Länge, d.h. wenigstens die Overhead - Struktur vorhanden?
  
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
    CheckFrameLengthAndCRC = False          ' Soll- und Ist-Länge verschieden
  End If

Else
  CheckFrameLengthAndCRC = False            ' Frame zu kurz, auf jeden Fall ungültig
End If                                      ' Ende Längen-Vergleich

End Function                            ' Ende CheckFrameLengthAndCRC



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
  
End Function                                                  ' Ende FnGetByteFromFrame





'///////////////////////////////////////////////////////////////////////////////////////////////////////////////
'//                                                                                                           //
'// Ende Kommunikationsfunktionen                                                                             //
'//                                                                                                           //
'// Die folgenden Funktionen sind v.a. für GUI und Datei - Handling zuständig                                                                             //
'//                                                                                                           //
'///////////////////////////////////////////////////////////////////////////////////////////////////////////////






'/////////////////////////////////////// Menü und so ///////////////////////////////////////////////////////////


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'neue Kennlinie erzeugen                                                                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub new_profile_Click()
    
  Dim Frage As Variant
  Dim strAux As String                          ' Hilfs - String
  Dim strFileNameAndPath As String              ' Datei Name und Pfad
  Dim bEnableReset As Boolean                   ' Flag für "Reset/New erlaubt"
  
  If g_bProfileChanged = True Then              ' schon was "speicherwürdiges" vorhanden?
    If (AskForSaveProfile = True) Then          ' neu laden freigegeben?
      bEnableReset = True                       ' ja, Reset geht klar
    End If
  Else                                          ' nöö, noch nix zu speichern
    bEnableReset = True                         ' ja, Reset geht klar
  End If
    
  
  If (bEnableReset = True) Then
    g_strCommentText = ""                       ' Kommentar-Text löschen
    strFileNameAndPath = App.Path & "\defneu.ini"                         ' Default-Neu-Datei
    
    If (GetProfileFromFile(strFileNameAndPath, False) = STATE_OK) Then    ' vom Ini-File laden
    Else
      Call InitCapsuleToDefault                 ' "hart" codierte Defaults laden
    End If
    
    UpdateGUI (UPDATE_ALL)                      ' neu anzeigen
    
    g_bProfileChanged = False                   ' jetzt nix mehr "wollen sie speichern"
    g_strProfileName = "N/A"                    ' keine benannte Kennlinie
    strAux = LoadResString(STR_ID_APP_TITLE + g_lLocaleOffset) & " - " & LoadResString(STR_ID_TXT_NEW_PROF + g_lLocaleOffset)
    kennlinie.Caption = CompressTextToMaxLength(strAux, kennlinie.Width, kennlinie)   ' bg018
    

  End If
  
End Sub                       ' Ende new_profile_click




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinie aus Datei laden, Menü - Klick - Funktion; per Dialog wird Name/Pfad der Datei erfragt               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Private Sub load_profile_Click()
    
Dim lErrOk As Long
Dim strAux As String                            ' Hilfs-String
Dim strFileNameAndPath As String                ' Datei Name und Pfad
Dim bEnableLoading                              ' Flag "laden erlaubt"


  bEnableLoading = False                        ' laden vorerst verhindern
  
  If g_bProfileChanged = True Then              ' schon was "speicherwürdiges" vorhanden?
    If (AskForSaveProfile = True) Then          ' neu laden freigegeben?
      bEnableLoading = True                     ' ja, laden geht klar
    End If
  Else                                          ' nöö, noch nix zu speichern
    bEnableLoading = True                       ' ja, laden geht klar
  End If


  If (bEnableLoading = True) Then               ' wirklich laden

    datei_fenster1.InitDir = CurDir()           ' Fokus auf aktuelles Arbeitsverzeichnis setzen
    
    strFileNameAndPath = ""                     ' zu ladende Datei per Dialog abfragen
    strFileNameAndPath = GetFileName(FILE_LOAD, lErrOk)
      
    If (lErrOk <> NO_ERROR) Then                ' Fehler bei Dateiname/Pfad
      Call DisplayErrOrMessage(lErrOk, "")      ' Fehlermeldung anzeigen
    Else
      g_strCommentText = ""                     ' Kommentar-Text löschen
      If (GetProfileFromFile(strFileNameAndPath, False) = STATE_OK) Then ' Kennlinie aus Datei lesen
        strAux = LoadResString(STR_ID_APP_TITLE + g_lLocaleOffset) & " - " & strFileNameAndPath
        kennlinie.Caption = CompressTextToMaxLength(strAux, kennlinie.Width, kennlinie)   ' bg018
        g_strProfileName = ExtractFileNameFromPathFileString(strFileNameAndPath)  ' Name der Kennlinie merken
      Else
        Call DisplayErrOrMessage(ERROR_FILE_FORMAT, strFileNameAndPath)           ' Fehlermeldung anzeigen
      End If
      Call UpdateGUI(UPDATE_ALL)                ' komplett neu anzeigen
    End If
  
    g_bProfileChanged = False                   ' jetzt speichern nicht nötig
  
  End If

End Sub                              ' Ende load_profile_Click




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinie in Datei speichern, Menü - Klick - Funktion; per Dialog wird Name/Pfad der Datei erfragt            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub save_profile_Click()
  
Dim lErrOk As Long
Dim strFileNameAndPath As String              ' Datei Name und Pfad
Dim strAux As String                          ' Hilfs-String
  
  datei_fenster1.InitDir = CurDir()           ' Fokus auf aktuelles Arbeitsverzeichnis setzen
  
  strFileNameAndPath = ""                                    ' Datei - Name zunächst leer
  strFileNameAndPath = GetFileName(FILE_SAVE, lErrOk)        ' Datei - Name und Pfad ermitteln
  
  If (lErrOk <> NO_ERROR) Then                ' Fehler bei Dateiname/Pfad
    Call DisplayErrOrMessage(lErrOk, "")      ' Fehlermeldung anzeigen
  Else                                        ' Pfad / Dateiname in Ordnung
    If (PutProfileToFile(strFileNameAndPath) = True) Then         ' Speichern erfolgreich?
      strAux = LoadResString(STR_ID_APP_TITLE + g_lLocaleOffset) & " - " & strFileNameAndPath  ' ja, neuen Dateinamen anzeigen
      kennlinie.Caption = CompressTextToMaxLength(strAux, kennlinie.Width, kennlinie)   ' bg018
      g_strProfileName = ExtractFileNameFromPathFileString(strFileNameAndPath)  ' Name der Kennlinie merken
      g_bProfileChanged = False               ' jetzt frisch gespeichert
    End If
  End If

End Sub                             ' Ende save_profile_click



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinie zum Gerät schicken                                                                                  '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Profile_to_Click()
   
  If MSComm1.PortOpen = False Then
    Call DisplayErrOrMessage(NO_CONNECTION_TO_NLG, "")
  Else
    Call SendReceiveProfile(PROFILE_SET)
  End If
  
End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Ladekennlinie von NLG zurücklesen                                                             '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub profile_from_Click()
Dim strAux As String                            ' Hilfs-String
    
  If MSComm1.PortOpen = False Then                                ' keine Verbindung, nix geht
    Call DisplayErrOrMessage(NO_CONNECTION_TO_NLG, "")
  Else                                                            ' Verbindung i.O.
    If g_bProfileChanged = True Then                              ' schon was "speicherwürdiges" vorhanden?
      If (AskForSaveProfile = True) Then                          ' neu laden freigegeben?
        Call SendReceiveProfile(PROFILE_GET)
        Call UpdateGUI(UPDATE_ALL)                                ' Gesamte Anzeige aus Datenkapsel neu aufbauen
      End If
    Else                                                          ' nöö, noch nix zu speichern
      Call SendReceiveProfile(PROFILE_GET)
      Call UpdateGUI(UPDATE_ALL)                                  ' Gesamte Anzeige aus Datenkapsel neu aufbauen
      g_bProfileChanged = True                                    ' Jetzt hamma wahrscheinlich was speicherwürdiges
    End If
  End If
  
  strAux = LoadResString(STR_ID_APP_TITLE + g_lLocaleOffset) & " - " & LoadResString(STR_ID_TXT_CURR_PROF + g_lLocaleOffset)
  kennlinie.Caption = CompressTextToMaxLength(strAux, kennlinie.Width, kennlinie)   ' bg018
  g_strProfileName = "N/A"                                        ' Kennlinie vom Gerät, Name nicht bekannt

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Schiesst die Kommunikation und das Programm ab, wenn per Menü beendet                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Quitter_Click()
     
  If MSComm1.PortOpen = True Then
    Call DisconnectNLG                            ' Gerät anständig ausloggen
  End If
  
  Unload Me

End Sub





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Verbindung mit NLG aufnehmen, d.h. Comm - Port prüfen / öffnen                                                '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub connect_nlg_Click()
    
  Dim bOk As Boolean                      ' connect erfolgreich?
  Dim iType As Integer                    ' NLG Typ als integer zusammenbasteln
  Dim lVers As Long                       ' NLG Firmware Version
  Dim strInfo As String                   ' Anzeige im Display
  
  Call ConnectNLG
  Call ReceiveReadOnlyInfo(bOk)
  
  If (bOk = True) Then                    ' wenn connect erfolgreich, dann..
    
    strInfo = "NLG "                      ' NLG-Typ holen und auswerten (X von "NLG 51X")
    iType = FnMin((NLG_MIN_KLASSE + (FnLoByteOfLong(GetDataFromCapsule(0, INF_NLG_TYP_51X)))), NLG_MAX_KLASSE)
    strInfo = strInfo & CStr(iType)       ' NLG-Typ anzeigen (aktuell: zwischen 510 und 516)
    
    NLGTyp.Text = strInfo                 ' NLG-Typ anzeigen
    NLGTyp.Refresh
  
    lVers = FnMin(GetDataFromCapsule(0, INF_FIRMWR_VERS), MAX_UINT)
    If (lVers < REQUIRED_FIRMWARE_VERSION) Then   ' Firmware passt nicht
      Call DisplayErrOrMessage(OLD_FIRMWARE, "")  ' "Firmware ev. inkompatibel"
    End If
  
  End If
    
End Sub                             ' Ende connect_nlg_Click



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Verbindung mit NLG abbrechen, d.h. Comm - Port schliessen und Menüs entsprechend setzen                       '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub disconnect_nlg_Click()
    
  Dim strTmp As String                            ' Anzeige im NLG-Feld
  
  If (MSComm1.PortOpen = True) Then
    Call DisconnectNLG                            ' Gerät anständig ausloggen
  Else
    If (SWITCH_DEBUG = True) Then
      MsgBox ("Comm1 noch nicht offen")
    End If
  End If
    
End Sub                                   ' Ende disconnect_nlg_Click




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Geräte - Info holen, ist im Gerät als Vorspann zur Kennlinie abgelegt                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Private Sub info_nlg_Click()
                                                                                                                '
  Dim bOk     As Boolean
  Dim lType   As Long
  Dim strInfo As String
  
  Call ReceiveReadOnlyInfo(bOk)                                 ' die Read - Only Info vom NLG abfragen
  
  If (bOk) Then                                                 ' wenn Abfrage erfolgreich, Werte aus Datenkapsel holen

    strInfo = "NLG "                      ' NLG-Typ holen und auswerten (X von "NLG 51X")
    lType = FnMin((NLG_MIN_KLASSE + (FnLoByteOfLong(GetDataFromCapsule(0, INF_NLG_TYP_51X)))), NLG_MAX_KLASSE)
    strInfo = strInfo & CStr(lType)       ' NLG-Typ anzeigen (aktuell: zwischen 510 und 516)
    GeraeteInfo.TextGerTyp = strInfo

    strInfo = ""                          ' Serie - Nr LSB holen und auswerten
    lType = FnMin(GetDataFromCapsule(0, INF_SER_NBR_LSB), MAX_UINT)
    lType = lType + (FnMin(GetDataFromCapsule(0, INF_SER_NBR_MSB), 16) * (MAX_UINT + 1))  ' "Angst-Begrenzung" auf ca. 1 Million
    strInfo = strInfo & CStr(lType)
    GeraeteInfo.TextSerNr = strInfo       ' Serie - Nr anzeigen

                                          ' Produktionsdatum holen und auswerten
    strInfo = ""                          ' Tag hinzufügen
    strInfo = strInfo & FormatNumber(FnMin(FnHiByteOfLong(GetDataFromCapsule(0, INF_PROD_DD_MM_)), MAX_DAY), "00") & "."
                                          ' Monat hinzufügen
    strInfo = strInfo & Format(FnMin(FnLoByteOfLong(GetDataFromCapsule(0, INF_PROD_DD_MM_)), MAX_MONTH), "00") & "."
                                          ' Jahr hinzufügen
    lType = FnMin(GetDataFromCapsule(0, INF_PROD_YYYY__), MAX_YEAR)
    strInfo = strInfo & CStr(lType)
    GeraeteInfo.TextProdDat = strInfo     ' Produktionsdatum anzeigen

    strInfo = ""                          ' Firmware Version holen und auswerten
    lType = FnMin(GetDataFromCapsule(0, INF_FIRMWR_VERS), 9999)
    strInfo = strInfo & Format(lType / 100, "0.00")             ' Format "x.yy.zz"
    lType = FnMin(GetDataFromCapsule(0, INF_FIRMWR_REV_), 100)  ' wenn > 99 ist's Müll ->  nicht anzeigen
    If (lType < 100) Then
      strInfo = strInfo & "." & Format(lType, "00")             ' sonst Revisionsindex auch anzeigen
    End If
    GeraeteInfo.TextFirmwareVers = strInfo  ' Firmware Version anzeigen
  
  Else      ' "Fehler" in allen Feldern anzeigen
    
    GeraeteInfo.TextGerTyp = LoadResString(STR_ID_ANY_FAULT + g_lLocaleOffset)
    GeraeteInfo.TextSerNr = LoadResString(STR_ID_ANY_FAULT + g_lLocaleOffset)
    GeraeteInfo.TextProdDat = LoadResString(STR_ID_ANY_FAULT + g_lLocaleOffset)
    GeraeteInfo.TextFirmwareVers = LoadResString(STR_ID_ANY_FAULT + g_lLocaleOffset)
  
  End If
  
  GeraeteInfo.Show vbModal                  ' Info - Böxli zeigen, modal (Gegenteil: vbModeless)

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kommentar-Text des gerade programmierten Geräts anzeigen, bei Bedarf die ganze Information drucken lassen..   '
' (Datei-Name, Datum, Pfad, Geräte-Eigenschaften, Teile der Kennlinie und ggf. Kommentar.                       '
' v.a. für Doku-Zwecke in der Produktion gedacht                                                                '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub show_comment_Click()

  If (SWITCH_DEBUG = True) Then
    MsgBox ("Hier kommt die Druckfunktion für Produktiönler rein")
  End If
  
  DlgEigenschGeraetBatt.Show

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Hier wird der Help als Thema bzw. als Hotkey angesprochen                                                                       '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub HelpF1_Click()

  datei_fenster1.HelpFile = App.HelpFile                    ' HelpFile wird im Hauptprogramm festgelegt
  
  If Me.ActiveControl.HelpContextID <> 0 Then                             ' brauchbare Kontext-ID? Wenn ja, dann..
    datei_fenster1.HelpCommand = cdlHelpContext             '..kontext-sensitive Hilfe anwerfen,..
    datei_fenster1.HelpContext = Me.ActiveControl.HelpContextID           '..Kontext vom aktuellen Control holen
  Else                                                      ' keine brauchbare Kontext-ID, also Inhaltsverzeichnis anzeigen
    datei_fenster1.HelpCommand = &HB                        ' Sch...VB, hier müsste eigentlich sowas wie "Contents" stehen, aber das geht..
  End If                                                    '..irgendwie nicht, deshalb muss der Wert "&HB" verwendet werden
  
  datei_fenster1.ShowHelp                                   ' Hilfe anzeigen

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Hier wird der Inhalt des Help angezeigt                                                                       '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub HelpContents_Click()

  datei_fenster1.HelpFile = App.HelpFile                  ' HelpFile wird im Hauptprogramm festgelegt
  datei_fenster1.HelpCommand = &HB                        ' Sch...VB, hier müsste eigentlich sowas wie "Contents" stehen, aber das geht..
                                                          '..irgendwie nicht, deshalb muss der Wert "&HB" verwendet werden
  datei_fenster1.ShowHelp                                 ' Hilfe anzeigen

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' das About / Info - Fenster                                                                                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub HelpAbout_Click()
  
  frmAbout.Caption = LoadResString(STR_ID_ABOUT_BOX_TITLE + g_lLocaleOffset)
  frmAbout.Show vbModal

End Sub



'/////////////////////////////////////// Ende Menü /////////////////////////////////////////////////////////////







'/////////////////////////////////// die Controls der "allgemeinen Einstellungen" //////////////////////////////



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Betriebsart Standard / Master / Booster wählen, in die Datenkapsel übernehmen; ggf. andere Controls anpassen  '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub ComboBetriebsart_Click()
              
  If ComboBetriebsart.ListIndex <> C_STANDARD Then                  ' Booster / CAN - Betrieb
    sCurrentHeight = BOOSTER_HEIGHT                                 ' aktuelle Höhe merken
    kennlinie.Height = BOOSTER_HEIGHT                               ' Abschnitte / U-I-Phasen Ctrls verbergen durch Fenster verkleinern
    If ComboBetriebsart.ListIndex = C_BOOSTER Then                  ' Booster - Betrieb
      TextMinIBoost.Visible = True
      TextMaxIBoost.Visible = True
      TextMaxINetz.Visible = True
      CWNumEdit1(GUI_BOOST_MIN).Visible = True                      ' Control ext Strom für min Booster Leistung anzeigen
      CWNumEdit1(GUI_BOOST_MAX).Visible = True                      ' Control ext Strom für max Booster Leistung anzeigen
      CWNumEdit1(GUI_MAX_I_NEZ).Visible = True                      ' Control max Netz-Strom anzeigen
    Else                                                            ' CAN-Betrieb
      TextMinIBoost.Visible = False
      TextMaxIBoost.Visible = False
      TextMaxINetz.Visible = False
      CWNumEdit1(GUI_BOOST_MIN).Visible = False                     ' Control ext Strom für min Booster Leistung verbergen
      CWNumEdit1(GUI_BOOST_MAX).Visible = False                     ' Control ext Strom für max Booster Leistung verbergen
      CWNumEdit1(GUI_MAX_I_NEZ).Visible = False                     ' Control max Netz-Strom verbergen
    End If
  Else                                                              ' Master / Standard - Betrieb
    sCurrentHeight = NORMAL_HEIGHT                                  ' aktuelle Höhe merken
    kennlinie.Height = NORMAL_HEIGHT                                ' Abschnitte / U-I-Phasen Ctrls anzeigen
    TextMinIBoost.Visible = False
    TextMaxIBoost.Visible = False
    TextMaxINetz.Visible = True
    CWNumEdit1(GUI_BOOST_MIN).Visible = False                       ' Control ext Strom für min Booster Leistung verbergen
    CWNumEdit1(GUI_BOOST_MAX).Visible = False                       ' Control ext Strom für max Booster Leistung verbergen
    CWNumEdit1(GUI_MAX_I_NEZ).Visible = True                        ' Control max Netz-Strom anzeigen
  End If
  
  ' Parameter "NLG Betriebsart" in Kapsel schreiben
  Call PutGuiValToCapsule(lGuiParBetrArt, FnLimit(ComboBetriebsart.ListIndex, C_STANDARD, C_CAN))

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fokus-Verlust eines "allgemeinen" GUI Controls: prüfen, ob Batteriespannung, ggf. Fensterli anzeigen          '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub CWNumEdit1_Validate(iIndex As Integer, KeepFocus As Boolean)

  If (iIndex = GUI_BATT_NENN) Then                ' Spezialfall Batteriespannung, mit Abfrage "alles anpassen?" gekoppelt
    If (FrmSpgKonvert.Visible = True) Then        ' Fokus - Verlust nur erlaubt, wenn Konvertier - Fensterli geschlossen
      FrmSpgKonvert.WindowState = vbNormal        ' Sonst setzen wir's auf Normalgrösse und..
      FrmSpgKonvert.ZOrder                        '..bringen's in den Vordergrund
      KeepFocus = True                            ' Fokus bleibt hier
    End If
  End If

  If (iIndex = GUI_BOOST_MIN) Then                ' bg026 zulässige Booster-Grenzen festlegen
    CWNumEdit1(GUI_BOOST_MAX).Minimum = CWNumEdit1(GUI_BOOST_MIN).Value + MIN_BOOSTER_RAMPE
  End If

  If (iIndex = GUI_BOOST_MAX) Then                ' bg026 zulässige Booster-Grenzen festlegen
    CWNumEdit1(GUI_BOOST_MIN).Maximum = CWNumEdit1(GUI_BOOST_MAX).Value - MIN_BOOSTER_RAMPE
  End If


End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Einen geänderten Wert der "Allgemeinen Einstellungen" in die Datenkapsel übernehmen                           '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub CWNumEdit1_ValueChanged(iIndex As Integer, vValue As Variant, vPreviousValue As Variant, ByVal bOutOfRange As Boolean)
  
Dim fVoltAdaptRatio As Single                   ' Spannungsverhältnis
Dim lNewValue As Long                           ' neuer Wert
Dim lActProp As Long                            ' aktuelle Eigenschaft
Dim iCount As Integer, iCount2 As Integer       ' Schleifen-Zähler
Dim bChangeBack As Boolean                      ' Backup vom "Profile Changed" Flag
 
  If (bGUIToCapsule = True) Then                ' Schreiben in Kapsel erlaubt? Wenn nein (Fenster-Refresh), tu nix
    
    '----------------------- Funktion für Nennspannung, ist speziell ----------------
    
    If (iIndex = GUI_BATT_NENN) Then            ' Spezialfall Batterie - Nennspannung: "Du wolle alle U's anpasse?"
      
      If (FrmSpgKonvert.Visible = False) Then   ' mach nur was, wenn das Ding nicht eh schon angezeigt wird
        
        g_lMsgVoltConversionState = STATE_WAIT_FOR_REACTION   ' Warteposition einnehmen, bis Benutzer sich für was entschieden hat
        bChangeBack = g_bProfileChanged         ' Zustand von "Profile Changed?" sichern
        
        FrmSpgKonvert.Show vbModal   ' Fensterli "Batt. Nennspannung geändert, wollen alle spannungen anpassen?" (anpassen / gleich lassen / abbrechen)
        
        Do                                      ' warten auf Benutzereingabe
          ret = DoEvents()                      ' Halb-Busy-Wait ("kooperatives Multitasking")
        Loop Until (g_lMsgVoltConversionState <> STATE_WAIT_FOR_REACTION)   ' Benutzer hat was eingegeben
        
        Select Case g_lMsgVoltConversionState   ' was hat Benutzer gewählt?
        
          Case STATE_ADAPT_ALL                  ' will alles anpassen lassen, no denn...
            
            Call PutGuiValToCapsule(lGuiParArrayAllg(iIndex), vValue)   ' Nominalspannung in die Datenkapsel
            ' Die folgenden Berechnungen usw. haben nix mit GUI zu tun, also direkt auf Datenkapsel zugreifen
            fVoltAdaptRatio = vValue / FnMax(vPreviousValue, MIN_NOM_BATT_VOLT)       ' Verhältnis ausrechnen
                                                ' Wert von Ueberspannung holen, neuen Wert berechnen und zurückschreiben
            lNewValue = fVoltAdaptRatio * GetDataFromCapsule(FnHiByteOfLong(BAT_SPANNG_UEBER), FnLoByteOfLong(BAT_SPANNG_UEBER))
            Call PutDataToCapsule(FnHiByteOfLong(BAT_SPANNG_UEBER), FnLoByteOfLong(BAT_SPANNG_UEBER), lNewValue)
            
            ' jetzt alle LadeAbschnitte durchtöggeln und anpassen:
            For iCount = 0 To (ANZ_LADEABSCHNITTE - 1)
              
              lActProp = MAX_U_PH_SPANN + iCount * LAENGE_ABSCHN   ' max. Ladespg. U-Phase des aktuellen Abschnitts
              lNewValue = fVoltAdaptRatio * GetDataFromCapsule(FnHiByteOfLong(lActProp), FnLoByteOfLong(lActProp))
              Call PutDataToCapsule(FnHiByteOfLong(lActProp), FnLoByteOfLong(lActProp), lNewValue)
            
              ' alle Bedingungen dieses Abschnitts durchtöggeln und wenn U-Bedingungen vorhanden deren Wert anpassen:
              For iCount2 = 0 To (ANZ_WECHSEL_BEDINGUNGEN - 1)
                lActProp = TYP_VORW_BED_1 + iCount * LAENGE_ABSCHN + iCount2 * OFFSET_BEDING   ' Bedingungs-Typ holen
                lNewValue = GetDataFromCapsule(FnHiByteOfLong(lActProp), FnLoByteOfLong(lActProp))
                
                If (lNewValue = COND_ID_SPANG_UEB) Or (lNewValue = COND_ID_SPG5_MIN_UNT) Then   ' Wenn's ne U-Bedingung..
                  lActProp = lActProp + 1       '..dann ihren Wert holen (Prop+1), anpassen und wieder speichern
                  lNewValue = fVoltAdaptRatio * GetDataFromCapsule(FnHiByteOfLong(lActProp), FnLoByteOfLong(lActProp))
                  Call PutDataToCapsule(FnHiByteOfLong(lActProp), FnLoByteOfLong(lActProp), lNewValue)
                End If
              Next iCount2                      ' nächste Bedingung
            
            Next iCount                         ' nächster Abschnitt
          
          
          Case STATE_CHANGE_NOM_ONLY            ' nur Nominalspannung ändern
            Call PutGuiValToCapsule(lGuiParArrayAllg(iIndex), vValue)   ' ab in die Datenkapsel damit

          Case STATE_CANCEL                     ' gar nix machen, alten Wert wiederherstellen
            Call PutGuiValToCapsule(lGuiParArrayAllg(iIndex), vPreviousValue)   ' ab in die Datenkapsel damit
            g_bProfileChanged = bChangeBack
          
          Case Else                             ' alles andere gilt als CANCEL / "gar nix machen"
            If SWITCH_DEBUG = True Then
              MsgBox ("unbekannte Aktion beim Spannungskonvertieren")
            End If
            Call PutGuiValToCapsule(lGuiParArrayAllg(iIndex), vPreviousValue)   ' ab in die Datenkapsel damit
            g_bProfileChanged = bChangeBack
        
        End Select
        
        UpdateGUI (UPDATE_ALL)                  ' alles neu darstellen
      
      Else                                      ' das Ding ist vielleicht im Hintergrund oder so? dann..
        
        FrmSpgKonvert.WindowState = vbNormal    '..setzen wir's auf Normalgrösse und..
        FrmSpgKonvert.ZOrder                    '..bringen's in den Vordergrund
      
      End If
    
    '----------------------- Funktion für (fast) alle anderen Controls der Allg. Einstellungen ----------------
    
    Else                                        ' Normalfall: genau 1 Wert in Datenkapsel schreiben
    
      If (bInvalidCWNumEd1 = False) Then  ' "Semaphor" offen? ("CWNumEdit1-Werte gerade nicht gültig", Hysterese etc.)
        Call PutGuiValToCapsule(lGuiParArrayAllg(iIndex), vValue)   ' ab in die Datenkapsel damit
      End If
  
    End If            ' Ende Nennspanng / andere Controls
  
  End If          ' Ende "schreiben in Kapsel erlaubt?"

End Sub       ' Ende CWNumEdit1_ValueChanged



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Art der Reduktion bei Erwärmung geändert: Parameter-Anzeige und Controls anpassen, in Datenkapsel schreiben   '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Reduktions_art_Click()

Dim lTmp As Long

  bInvalidCWNumEd1 = True                                   ' während Ctrl Umbau keine Daten speichern
   
  If Reduktions_art.ListIndex = C_DERATING Then             ' Derating gewählt
    
    lGuiParArrayAllg(GUI_DERAT_HYS) = BATT_TEMP_DERATE      ' ID für Derating Wert in Amp/°C
    lGuiParArrayAllg(GUI_BEZG_HYST) = TEMP_DERATE_NULL      ' ID für Bezugstemperatur für Derating auf 0
    bInvalidCWNumEd1 = True                                 ' solange Ctrl umgebaut wird, keine Daten speichern
    TextBatTempDerat.Text = LoadResString(STR_ID_GEN_BAT_TEMP_DERAT + g_lLocaleOffset)   ' alt "Batterie-Temperatur-Derating   [A/°C]"
    TextBezTempDerat.Text = LoadResString(STR_ID_GEN_REF_DERAT_ZERO + g_lLocaleOffset) ' alt "Bezugstemperatur für Derating auf Null   [°C]"
    CWNumEdit1(GUI_DERAT_HYS).Minimum = -10                 ' Min / Max für Derating
    CWNumEdit1(GUI_DERAT_HYS).Maximum = 0
    
  Else                                                      ' Hysterese gewählt
    
    lGuiParArrayAllg(GUI_DERAT_HYS) = BAT_HYSTERES_EIN      ' Temperatur für Ein
    lGuiParArrayAllg(GUI_BEZG_HYST) = BAT_HYSTERES_AUS      ' Temperatur für Aus
    TextBatTempDerat.Text = LoadResString(STR_ID_GEN_TEMP_HYSTER_ON + g_lLocaleOffset)   ' "Einschaltpunkt für Batterietemp.-Hysterese   [°C]"
    TextBezTempDerat.Text = LoadResString(STR_ID_GEN_TEMP_HYSTER_OFF + g_lLocaleOffset) '  = "Ausschaltpunkt für Batterietemp.-Hysterese   [°C]"
    CWNumEdit1(GUI_DERAT_HYS).Minimum = 19                   ' Min / Max für Hysterese
    CWNumEdit1(GUI_DERAT_HYS).Maximum = 59
  
  End If
    
  bInvalidCWNumEd1 = False                                   ' Ctrl ist fertig, Daten wieder speichern
  
  ' Parameter "Reduktions - Art" in Kapsel schreiben
  Call PutGuiValToCapsule(lGuiParRedArt, FnLimit(Reduktions_art.ListIndex, C_DERATING, C_HYSTERESE))
  
  ' allfällig vorhandene Werte für Derating / Hysterese in Controls laden
  ' "Derating %/°C" oder "Hysterese Ein"
  CWNumEdit1(GUI_DERAT_HYS).Value = GetGuiValFromCapsule(lGuiParArrayAllg(GUI_DERAT_HYS))

  '"Bezugstemperatur" oder "Hysterese Aus"
  CWNumEdit1(GUI_BEZG_HYST).Value = GetGuiValFromCapsule(lGuiParArrayAllg(GUI_BEZG_HYST))
  
End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Einen (der 3) Temperatur - Sensoren aktivieren / deaktivieren, in die Datenkapsel übernehmen                  '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub CheckTempSens_Click(iIndex As Integer)
  
  ' Parameter "Aktive Temperatur-Sensoren" in Kapsel schreiben
    
  Dim ySensTemp As Byte                           ' Zustand des angeklickten Sensors
  Dim lSensZustand As Long                        ' aktueller Zustand der 3 Sensoren
  Dim lProp As Long                               ' Property
    
  
  lProp = lGuiParAktTempSens(iIndex)              ' Property feststellen
  lSensZustand = GetGuiValFromCapsule(lProp)      ' aktuellen Wert der Property feststellen (enthält alle 3 Sensoren)
  ySensTemp = FnGetCharOfNumber(lSensZustand, (iIndex))     ' aktuellen Zustand dieses Sensors aus Property holen
  
  If (ySensTemp <> CheckTempSens(iIndex).Value) Then        ' wenn Checkbox neuen Wert enthält, dann..
    Call PutCharToNumber(lSensZustand, CheckTempSens(iIndex).Value, iIndex)     ' Zustand wieder in Property schreiben
    Call PutGuiValToCapsule(lProp, lSensZustand)            ' neuen Wert in die Kapsel schreiben
    WriteValToGUI (lProp)                                   ' neuen Wert anzeigen
  End If
  
  UpdateGUI (UPDATE_TEMPSENS)                     ' temp-relevante controls enablen/disablen

End Sub



'/////////////////////////////////// Ende Controls "allgemeine Einstellungen" //////////////////////////////////





'/////////////////////////////////// die Controls der Abschnitte ///////////////////////////////////////////////


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Abschnitt der Kennlinie wählen                                                                                '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub ComboAbschnitt_Click()
   
  g_iAktuellerAbschnitt = (ComboAbschnitt.ListIndex) + 1
  
  If g_iAktuellerAbschnitt = 1 Then          ' Beim ersten Abschnitt kann man nicht rückwärts
    BedingTypRW.Enabled = False
    AbschnZurueck.Enabled = False
  Else
    BedingTypRW.Enabled = True
    AbschnZurueck.Enabled = True
  End If
  
  If g_iAktuellerAbschnitt = ANZ_LADEABSCHNITTE Then          ' Beim letzten Abschnitt kann man nicht vorwärts
    BedingTypVW.Enabled = False
    AbschnVor.Enabled = False
  Else
    BedingTypVW.Enabled = True
    AbschnVor.Enabled = True
  End If
  
  UpdateGUI (UPDATE_SECTION)
   
End Sub

  
  
  
  ' ------------------- Controls "generell zu diesem Abschnitt" -----------------------


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Ein Wert in den generellen Daten (max. Spannung/Strom in U/I-Phase, sowie Beitrag zur Überladung) zu diesem.. '
'..Abschnitt hat geändert, also in die Datenkapsel schreiben (Neu-Anzeige besorgt das Control selber)           '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub num1_ValueChanged(Index As Integer, Value As Variant, PreviousValue As Variant, ByVal OutOfRange As Boolean)
    
  If (bGUIToCapsule = True) Then              ' Schreiben in Kapsel erlaubt?
    Call PutGuiValToCapsule((lGuiGenAbEdBox(Index) + (g_iAktuellerAbschnitt - 1) * LAENGE_ABSCHN), Value)
  End If
    
End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Tastatur-Handler für die LED-Knöpfchen; wenn der Fokus drauf ist und man "SPACE" drückt, "schalten"..         '
'..sie einen Zustand weiter (wird auf "Click"-Funktion umgeleitet)                                                                                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub LEDButton_KeyPress(iIndex As Integer, iKey As Integer)

  If (iKey = Asc(" ")) Then                 ' Fokus auf LED
    Call LEDButton_Click(iIndex)            ' auf Klick - Ereignis umleiten
  End If

End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Die LED-Knöpfchen können aus (grau) sein, blinken oder leuchten; wenn man draufklickt, "schalten"..           '
'..sie einen Zustand weiter                                                                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub LEDButton_Click(iIndex As Integer)
    
  Dim yLedTemp As Byte                            ' Zustand der angeklickten LED
  Dim yPosition                                   ' Stelle, wo sich die LED befindet (in ihrer Dreiergruppe)
  Dim lLedZustand As Long                         ' aktueller Zustand der 3 LED's
  Dim lProp As Long                               ' Property
    
  
  lProp = lGuiStatusLeds(iIndex) + ((g_iAktuellerAbschnitt - 1) * LAENGE_ABSCHN)  ' Property ausrechnen
  
  lLedZustand = GetGuiValFromCapsule(lProp)       ' aktuellen Wert der Property feststellen (enthält alle 3 LED's)
  
  If (iIndex < GUI_U_PH_STR) Then                 ' Position der einzelnen LED feststellen, als Stelle im Property
    yPosition = (iIndex - GUI_I_PH_STR)
  Else
    yPosition = (iIndex - GUI_U_PH_STR)
  End If
  
  yLedTemp = FnGetCharOfNumber(lLedZustand, yPosition)    ' aktuellen Zustand dieser LED aus Property holen
  
  If (yLedTemp = LED_EIN) Then                    ' wenn LED Ein, ist der nächste Zustand "Aus"
    yLedTemp = LED_AUS
  Else
    yLedTemp = yLedTemp + 1                       ' sonst einen Zustand weiterschalten
  End If
  
  Call PutCharToNumber(lLedZustand, yLedTemp, yPosition)  ' Zustand wieder in Property schreiben
  
  Call PutGuiValToCapsule(lProp, lLedZustand)     ' neuen Wert in die Kapsel schreiben
  
  WriteValToGUI (lProp)                           ' neuen Wert anzeigen
    
End Sub                     ' Ende LEDButton_Click

  
  
  

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' eins der 4 Ankreuz - Kästchen mit den Bedingungen für "zurück zum Abschnitt 1" (aus/ein, Quittiersignale)..   '
'..geändert, also Parameter des aktuellen Abschnitts entsprechend ändern                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Check1_Click(Index As Integer)
  
  ' N.B.: um Komplikationen mit VB Extra-Würsten zu vermeiden, definieren wir Wahr als 1 und Falsch als 0

If (bGUIToCapsule = True) Then                        ' Schreiben in Kapsel erlaubt?

  If (Check1(Index).Value = vbChecked) Then           ' Kästchen wurde angekreuzt
    Call PutGuiValToCapsule((lGuiRetCheckBox(Index) + ((g_iAktuellerAbschnitt - 1) * LAENGE_ABSCHN)), BTRUE)  ' bg002 neue Variante: ab in die Datenkapsel damit
'    Mid(abschnitt_generell(g_iAktuellerAbschnitt).zumab1, Index, 1) = "1"
  Else                                                ' Kästchen wurde nicht angekreuzt
    Call PutGuiValToCapsule((lGuiRetCheckBox(Index) + ((g_iAktuellerAbschnitt - 1) * LAENGE_ABSCHN)), BFALSE)      ' bg002 neue Variante: ab in die Datenkapsel damit
'    Mid(abschnitt_generell(g_iAktuellerAbschnitt).zumab1, Index, 1) = "0"
  End If

End If

End Sub
  
  
  
  ' ------------------- Ende Controls "generell zu diesem Abschnitt" ------------------





  ' ------------------- Controls mit Wechsel - Bedingungen als Texte  -----------------



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' neue Vorwärts - Bedingung erstellen                                                           '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub BedingTypVW_DblClick()
    
  Dim iCount As Integer                           ' Zählvariable
  Dim iCondID As Integer                          ' Identifier der Bedingung
'  Dim iCheckCond As Integer                       ' identifier der zu überprüfenden Bedingung
  Dim iNbrFwd As Integer                          ' Anzahl bereits vorhandener Vorwärts - Bedingungen holen
  Dim iOffset As Integer                          ' ein veränderlicher Offset
  Dim bAddNewCondition As Boolean                 ' soll die neue Bedingung hinzugefügt werden?
  
' Daten direkt in Datenkapsel holen/schreiben, die Anzeige wird daraus aufgebaut

    bAddNewCondition = True                       ' neue Bedingung hinzufügen
    iOffset = (LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1))   ' Offset in der Struktur
    iCondID = BedingTypVW.ItemData(BedingTypVW.ListIndex)     ' Identifier der ausgewählten Bedingung holen
    
    ' --------- neue Bedingung möglich? --------
    
    iNbrFwd = FnMin(ANZ_WECHSEL_BEDINGUNGEN, GetGuiValFromCapsule(ANZ_VORWAE_BED + iOffset))  ' wieviele vorwärts - Bedingungen hamma schon?
    
    If (iNbrFwd < ANZ_WECHSEL_BEDINGUNGEN) Then   ' Max Anzahl Bedingungen noch nicht erreicht
      bAddNewCondition = True
    Else                                          ' Max Anzahl Bedingungen ist erreicht
      bAddNewCondition = False
      Call DisplayErrOrMessage(ALL_CONDITIONS_USED, "") ' "Keine weiteren Bedingungen möglich"
    End If

    ' ------- ja, neue Bedingung hinzufügen ------
    
    If (bAddNewCondition = True) Then                                 ' Bedingung hinzufügen
      iNbrFwd = iNbrFwd + 1                                           ' eine Vorwärts - Bedingung mehr
      iOffset = LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1)           ' Abschnitts - Offset
      Call PutGuiValToCapsule(ANZ_VORWAE_BED + iOffset, iNbrFwd)      ' neue Anz Bedingungen in die Datenkapsel
      
      iOffset = iOffset + 3 * (iNbrFwd - 1)                           ' Offset für Datenkapsel machen
      Call PutGuiValToCapsule(TYP_VORW_BED_1 + iOffset, iCondID)      ' Typ der neuen Vorwärts - Bedingung
      Call PutGuiValToCapsule(VAL_VORW_BED_1 + iOffset, 0)            ' Wert der neuen Bedingung auf 0 setzen
      If (iNbrFwd > 1) Then                                           ' wenn mindestens 2 Bedingungen, dann..
        Call PutGuiValToCapsule(AND_OR_VORW_12 + iOffset - 3, C_OR)       '..und/oder initialisieren
      End If
                                                                      
      UpdateGUI (UPDATE_SECTION)                                      ' neu anzeigen
    End If

End Sub                                 ' Ende BedingTypVW_DblClick



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Doppelklick auf bestehende Vorwärts-Bedingung (eins der 5 möglichen Kästchen) löscht diese    '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub BedVorwText_DblClick(Index As Integer)
  
  Call BedVorwText_loeschen(Index)

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Eins der 5 Felder für neue Vorwärts - Bedingungen löschen, werden über Index angesprochen     '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub BedVorwText_loeschen(iIndex As Integer)
    
  Dim iAnzVorwBeding As Integer                   ' anzahl aktuell vorhandene vorwärts - Bedingungen
  Dim iCount As Integer                           ' Schleifenzähler
  Dim iOffset As Integer                          ' Offset
  Dim lTempPar As Long                            ' temporäre Variable für Par / Wert
  Dim iTempScale As Integer                       ' temp Skalierung
  Dim lPropID As Long                             ' berechnete ID der Property
  
  iOffset = (LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1))         ' Offset in der Struktur
  
  iAnzVorwBeding = FnMin(ANZ_WECHSEL_BEDINGUNGEN, GetGuiValFromCapsule(ANZ_VORWAE_BED + iOffset)) ' Anzahl der Vorwärts-Parameter des aktuellen Abschnitts holen
                                                  
  iCount = iIndex
  While (iCount < iAnzVorwBeding)                 ' wenn nicht letzte Bedingung, alle nachfolgenden eins nach vorne schieben
    
    lTempPar = GetGuiValFromCapsule(TYP_VORW_BED_1 + iOffset + 3 * iCount)  ' Typ um eins nachziehen
    Call PutGuiValToCapsule(TYP_VORW_BED_1 + iOffset + 3 * (iCount - 1), lTempPar)
                                                                            ' Skalierung und Wert um eins nachziehen
    lPropID = VAL_VORW_BED_1 + iOffset + 3 * iCount                               ' berechnete ID "alte" Property
    lTempPar = GetGuiValFromCapsule(lPropID)                                      ' Wert holen
    iTempScale = GetScaleOfPar(FnHiByteOfLong(lPropID), FnLoByteOfLong(lPropID))  ' Skalierung holen
    lPropID = VAL_VORW_BED_1 + iOffset + 3 * (iCount - 1)                         ' berechnete ID "neue" Property
    Call PutScaleToPar(FnHiByteOfLong(lPropID), FnLoByteOfLong(lPropID), iTempScale)  ' Skalierung einschreiben
    Call PutGuiValToCapsule(lPropID, lTempPar)                                    ' Wert einschreiben
    
    If ((iAnzVorwBeding > 2) And (iCount < (ANZ_WECHSEL_BEDINGUNGEN - 1))) Then   ' Verknüpfung noch sinnvoll?
      lTempPar = GetGuiValFromCapsule(AND_OR_VORW_12 + iOffset + 3 * iCount)      ' Verknüpfung um eins nachziehen
      Call PutGuiValToCapsule(AND_OR_VORW_12 + iOffset + 3 * (iCount - 1), lTempPar)
    End If
    
    iCount = iCount + 1
  
  Wend
                                                  ' Anzahl Bedingungen um eins verkleinern, in die Datenkapsel schreiben
  Call PutGuiValToCapsule(ANZ_VORWAE_BED + iOffset, iAnzVorwBeding - 1)
    
  UpdateGUI (UPDATE_SECTION)

End Sub                               ' Ende BedVorwText_loeschen




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Oder / Und - Button für Vorwärts - Bedingungen, Wert verändert
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub AndOrButton_ValueChanged(iIndex As Integer, ByVal bValue As Boolean)
  
Dim iValue As Integer                                     ' "normierte" Variante des Bools
Dim lPropID As Long                                       ' Property ID des Knopfs

  If (bValue = True) Then                                 ' dank dämlicher Boolean-Behandlung durch VB setzen..
    iValue = BTRUE                                        '..wir das sicherheitshalber auf integer um
  Else
    iValue = BFALSE
  End If
  
  
  lPropID = lGuiAndOrVor(iIndex) + (LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1))   ' Prop ID
  Call PutGuiValToCapsule(lPropID, iValue)                ' neuen Wert in die Datenkapsel schreiben
  UpdateGUI (UPDATE_SECTION)                              ' Abschnitt neu anzeigen

End Sub






'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' neuen Wert für Vorwärts - Bedingung eingestellt, in Datenstruktur übernehmen                  '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub BedVorwWert_ValueChanged(iIndex As Integer, vValue As Variant, PreviousValue As Variant, ByVal OutOfRange As Boolean)

Dim lPropID As Long
  
  lPropID = lGuiWertVor(iIndex) + (LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1))   ' Prop ID
  Call PutGuiValToCapsule(lPropID, vValue)                ' neuen Wert in die Datenkapsel schreiben
  UpdateGUI (UPDATE_SECTION)                              ' Abschnitt neu anzeigen

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' einen Abschnitt vorwärts hüpfen                                                               '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub AbschnVor_Click()
  
  If g_iAktuellerAbschnitt < ANZ_LADEABSCHNITTE Then      ' vorwärts wechseln, wenn nicht letzter Abschnitt
    g_iAktuellerAbschnitt = g_iAktuellerAbschnitt + 1     ' ein Abschnitt vorwärts
    ComboAbschnitt.ListIndex = g_iAktuellerAbschnitt - 1  ' Combo - Box mit den Ladeabschnitten nachführen
    ComboAbschnitt.Refresh                                ' nachgeführte Box neu anzeigen
  End If

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' einen Abschnitt rückwärts hüpfen                                                              '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub AbschnZurueck_Click()
  
  If g_iAktuellerAbschnitt > 1 Then                       ' rückwärts wechseln, wenn nicht erster Abschnitt
    g_iAktuellerAbschnitt = g_iAktuellerAbschnitt - 1     ' ein Abschnitt zurück
    ComboAbschnitt.ListIndex = g_iAktuellerAbschnitt - 1  ' Combo - Box mit den Ladeabschnitten nachführen
    ComboAbschnitt.Refresh                              ' nachgeführte Box neu anzeigen
  End If

End Sub

  
  
  ' ------------------- Ende Controls mit Wechsel - Bedingungen als Texte  ------------



'/////////////////////////////////// Ende Controls der Abschnitte //////////////////////////////////////////////




'////////////////////////////////// GUI <-> Datenkapsel Anbindung /////////////////////////////////////////



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Wert eines Parameters von GUI - Control entgegennehmen, skalieren und in die Datenkapsel schreiben            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub PutGuiValToCapsule(ByVal lPropID As Long, ByVal vPutValue As Variant)
  
  Dim yHi As Byte, yLo As Byte                          ' Hi und Lo Byte der Prop-ID
  Dim iScale As Integer                                 ' Skalierungsfaktor
  
  
  yHi = FnHiByteOfLong(lPropID)                         ' High Byte Prop_ID des betroffenen Parameters bestimmen
  yLo = FnLoByteOfLong(lPropID)                         ' Low Byte Prop_ID des betroffenen Parameters bestimmen
  
  iScale = GetScaleOfPar(yHi, yLo)                      ' Skalierungsfaktor bestimmen
  
  If (iScale <> 0) Then                                 ' Skalierungsfaktor gültig
    Call PutDataToCapsule(yHi, yLo, CLng(iScale * vPutValue)) ' skalierten Wert in die Kapsel schreiben
    g_bProfileChanged = True                            ' jetzt ist die Kennlinie bereits verändert
  Else                                                  ' Skalierungsfaktor ungültig
    If (SWITCH_DEBUG = True) Then
      MsgBox ("Fehler in PutGuiValToCapsule")
    End If
  End If

End Sub                                       ' Ende PutGuiValToCapsule



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Wert eines Parameters aus Datenkapsel lesen, für GUI - Control aufbereiten (skalieren)                        '
'                                                                                                               '
' lProp:          zu lesende Property                                                                           '
' Return:         Wert der zu lesenden Property, als Variant                                                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function GetGuiValFromCapsule(ByVal lPropID As Long) As Variant
  
  Dim yHi As Byte, yLo As Byte                          ' Hi und Lo Byte der Prop-ID
  Dim iScale As Integer                                 ' Skalierungsfaktor
  
  yHi = FnHiByteOfLong(lPropID)                         ' High Byte Prop_ID des betroffenen Parameters bestimmen
  yLo = FnLoByteOfLong(lPropID)                         ' Low Byte Prop_ID des betroffenen Parameters bestimmen
  iScale = GetScaleOfPar(yHi, yLo)                      ' Skalierungsfaktor bestimmen
  
  GetGuiValFromCapsule = GetDataFromCapsule(yHi, yLo)   ' Wert aus der Kapsel holen
  If ((GetGuiValFromCapsule > STATE_ERROR) And (iScale <> 0)) Then   ' nur positive Werte, Skalierung nicht 0; sonst Fehler
    GetGuiValFromCapsule = GetGuiValFromCapsule / iScale  ' Wert auf "Original - Grösse" skalieren
  Else
    If SWITCH_DEBUG = True Then
      MsgBox ("Fehler bei GetGuiValFromCapsule")
    End If
  End If
End Function                      ' Ende GetGuiValFromCapsule



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Anzeige im GUI auffrischen: alle aktuell sichtbaren Properties neu anzeigen, z.B. nach Laden einer Kennlinie..'
'..aus einer Datei oder vom NLG                                                                                 '
'                                                                                                               '
' lPart   : was updaten? (alles, nur Abschnitt, nur Temp.Controls etc.)                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub UpdateGUI(lPart As Long)
  
  Dim bDisp    As Boolean                             ' Wert anzeigbar?, wird auch zum enablen/disablen von Controls missbraucht
  Dim yHi As Byte, yLo As Byte                        ' Hi und Lo Byte der Prop_ID des aktuellen Parameters
  Dim yHiPrev As Byte, yLoPrev As Byte                ' Prop_ID's des vorherigen Par's
  Dim lProp As Long                                   ' Prop-ID als Long Variable repräsentiert (->bequemer)
  Dim lAbschnMin As Long                              ' kleinste Prop_ID für aktuellen Abschnitt
  Dim lLocVal As Long                                 ' lokaler Wert, Hilfsvar
    
    bGUIToCapsule = False                             ' während des GUI-Updates (von Datenkapsel) keine Daten von GUI in..
                                                      '..Kapsel schreiben; verhindert Selbstläufer etc.
    yHiPrev = NOT_PAR                                 ' Am Anfang ist der "vorherige Par" der "NichtParameter" an Stelle 0
    yLoPrev = NOT_PAR                                 '
      
    lAbschnMin = OFFSET_ABSCHN + (LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1))  ' Startadresse des aktuellen Abschnitts
      
                                
                                
    ' ------------- falls "alles" oder "Abschnitt", dann Parameter durchzählen --------------
    If ((lPart = UPDATE_ALL) Or (lPart = UPDATE_SECTION)) Then
      
      While (GetNextPar(yHiPrev, yLoPrev, yHi, yLo) = True)
        
        lProp = FnLongOf2Bytes(yHi, yLo)                ' Property als Long
        
        'wenn Prop < grösste Allgemeine Prop und "alles anzeigen", oder Prop im aktuellen Abschnitt enthalten, dann anzeigen
        If (((lProp <= MAX_ALLG_PAR) And (lPart = UPDATE_ALL)) Or ((lProp >= lAbschnMin) And (lProp < (lAbschnMin + LAENGE_ABSCHN)))) Then
          bDisp = WriteValToGUI(lProp)
          
          If ((bDisp = False) And (lProp > MAX_RDONLY)) Then
            If (SWITCH_DEBUG = True) Then
              'MsgBox ("Property " & lProp & " ausserhalb des anzeigbaren Bereichs")
            End If
          End If
              
        End If                                          ' Ende "Prop im anzeigbaren Bereich"
        
        yHiPrev = yHi
        yLoPrev = yLo
      
      Wend
    End If
    ' ------------- Ende Parameter durchzählen --------------
    
    
    ' bg021 Spez. Erweiterung: alle Temperatur-Einstellungen disablen, wenn kein Temp.Sensor aktiv ist
    lLocVal = GetGuiValFromCapsule(lGuiParAktTempSens(GUI_AKT_TMP_1))  ' Wert für Sensoren holen (es werden immer alle 3 geliefert)
    If (lLocVal = C_OFF) Then           ' kein Temp.sensor angekreuzt..
      bDisp = False                     '..also disablen..
    Else                                '..sonst..
      bDisp = True                      '..enablen
    End If
                                        ' alle relevanten Controls entsprechend enablen/disablen
    CWNumEdit1(GUI_TEMP_KOEF).Enabled = bDisp   ' Temp.Koeff. Gasungsspg.
    CWNumEdit1(GUI_BEZUG_TMP).Enabled = bDisp   ' Bezugstemp. Spgs-Angaben
    Reduktions_art.Enabled = bDisp              ' Temp.Red.Art (Derating/Hysterese)
    CWNumEdit1(GUI_DERAT_HYS).Enabled = bDisp   ' Derating-Wert oder unterer Hyst.Punkt
    CWNumEdit1(GUI_BEZG_HYST).Enabled = bDisp   ' Derating-Bezugstemp. oder oberer Hyst.Punkt
    CWNumEdit1(GUI_TMP_UEBER).Enabled = bDisp   ' Schutzfunktion "Temp. über"
    ' Ende bg021

    ' bg026 Einstellgrenzen für Booster setzen
'    If (GetDataFromCapsule(0, BETR_MASTR_BOOST) = C_BOOSTER) Then
      CWNumEdit1(GUI_BOOST_MAX).Minimum = CWNumEdit1(GUI_BOOST_MIN).Value + MIN_BOOSTER_RAMPE
      CWNumEdit1(GUI_BOOST_MIN).Maximum = CWNumEdit1(GUI_BOOST_MAX).Value - MIN_BOOSTER_RAMPE
'    End If
    ' Ende bg026
    
    bGUIToCapsule = True                        ' Daten von GUI in Kapsel schreiben wieder erlaubt


End Sub                            ' --- Ende UpdateGUI



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' die verschiedenen Parameter mit den zugehörigen Controls des GUI verknüpfen, indem hier die entsprechenden..  '
'..Prop_ID's der Parameter zu den Indizes der Controls geschrieben werden; damit eine freie Zuordnung möglich.. '
'..ist und die Übersetzung in bei Richtungen leicht geht, wurde eine (Array-) Tabelle gewählt                   '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub InitPosPropArrays()
  
  ' die Allgemeinen Parameter
  lGuiParArrayAllg(GUI_BATT_NENN) = BATT_NENN_SPANNG    ' will heissen: Inhalt des Array-Elements 0 ist die Prop_ID der Batterie-Nennspg
  lGuiParArrayAllg(GUI_TEMP_KOEF) = TEMP_KOEF_GAS_SP
  lGuiParArrayAllg(GUI_BEZUG_TMP) = BEZUGS_TEMP_SPNG
  lGuiParArrayAllg(GUI_MAX_P_NEZ) = MAX_NETZ_LEISTNG
  lGuiParArrayAllg(GUI_MAX_I_NEZ) = MAX_NETZ_STROM__
  lGuiParArrayAllg(GUI_DERAT_HYS) = BATT_TEMP_DERATE    ' Temp. Derating oder Hysterese
  lGuiParArrayAllg(GUI_BEZG_HYST) = TEMP_DERATE_NULL    ' Bezugs Temp. oder Hysterese
  lGuiParArrayAllg(GUI_TMP_UEBER) = BATT_TEMP_UEBER_
  lGuiParArrayAllg(GUI_SPG_UEBER) = BAT_SPANNG_UEBER
  lGuiParArrayAllg(GUI_ENERG_UEB) = NETZ_ENERGIE_UEB
  lGuiParArrayAllg(GUI_LADEN_UEB) = BAT_LAD_MEHR_ALS
  lGuiParArrayAllg(GUI_LADE_ZEIT) = LAD_ZEIT_MER_ALS
  lGuiParArrayAllg(GUI_BOOST_MIN) = EXT_STROM_P_MIN_
  lGuiParArrayAllg(GUI_BOOST_MAX) = EXT_STROM_P_MAX_
  
  lGuiParAktTempSens(GUI_AKT_TMP_1) = AKTIVE_TEMP_SENS  ' Temp Sensor #1
  lGuiParAktTempSens(GUI_AKT_TMP_2) = AKTIVE_TEMP_SENS  ' Temp Sensor #2
  lGuiParAktTempSens(GUI_AKT_TMP_3) = AKTIVE_TEMP_SENS  ' Temp Sensor #3
  
  lGuiParRedArt = TEMP_REDUKT_ART_                      ' Temperatur-Reduktions-Art ist in ComboBox dargestellt
  
  lGuiParBetrArt = BETR_MASTR_BOOST                     ' NLG - Betriebsart Standard / Master / Booster
  
  ' Achtung: diese Properties sind in jedem Abschnitt vorhanden, mit jeweils erhöhter Nummer; die untenstehenden..
  '..Zuordnungen GELTEN NUR für den ABSCHNITT 1  !
  
  ' die "Generell zu diesem Abschnitt" Parameter
  ' die U/I Phasen-Edits
  lGuiGenAbEdBox(GUI_I_PH_MAX) = MAX_I_PH_STROM
  lGuiGenAbEdBox(GUI_U_PH_MAX) = MAX_U_PH_SPANN
  lGuiGenAbEdBox(GUI_I_PH_UEB) = UEB_I_PH_STROM
  lGuiGenAbEdBox(GUI_U_PH_UEB) = UEB_U_PH_SPANN

  ' die Status Leds
  lGuiStatusLeds(GUI_I_PH_STL) = I_STATUS_LEDS_   ' I links
  lGuiStatusLeds(GUI_I_PH_STM) = I_STATUS_LEDS_   ' I mitte
  lGuiStatusLeds(GUI_I_PH_STR) = I_STATUS_LEDS_   ' I rechts
  lGuiStatusLeds(GUI_U_PH_STL) = U_STATUS_LEDS_   ' U links
  lGuiStatusLeds(GUI_U_PH_STM) = U_STATUS_LEDS_   ' U mitte
  lGuiStatusLeds(GUI_U_PH_STR) = U_STATUS_LEDS_   ' U rechts
  
  ' die "zurück zu Abschnitt 1" Checkboxen
  lGuiRetCheckBox(GUI_U_PH_AEG) = ZU_AB1_EIN_AUS  ' Aus / Ein -> geht
  lGuiRetCheckBox(GUI_U_PH_SG1) = ZU_AB1_QUIT_1_  ' bei Quit 1
  lGuiRetCheckBox(GUI_U_PH_SG2) = ZU_AB1_QUIT_2_  ' bei Quit 2
  lGuiRetCheckBox(GUI_U_PH_S12) = ZU_AB1_QUIT_12  ' bei Quit 1+2

  
  ' die "Abschnitt - Wechsel" Parameter
  lGuiTypVor(GUI_VW_BED_1) = TYP_VORW_BED_1       ' Typ der 1. vorwärts Bedingung
  lGuiWertVor(GUI_VW_VAL_1) = VAL_VORW_BED_1      ' Wert der 1. vorwärts Bedingung
  lGuiAndOrVor(GUI_VW_AO_12) = AND_OR_VORW_12     ' Verknüpfung And/Or vorwärts Bedingungen 1 und 2
  lGuiTypVor(GUI_VW_BED_2) = TYP_VORW_BED_2       ' Typ der 2. vorwärts Bedingung
  lGuiWertVor(GUI_VW_VAL_2) = VAL_VORW_BED_2      ' Wert der 2. vorwärts Bedingung
  lGuiAndOrVor(GUI_VW_AO_23) = AND_OR_VORW_23     ' Verknüpfung And/Or vorwärts Bedingungen 2 und 3
  lGuiTypVor(GUI_VW_BED_3) = TYP_VORW_BED_3       ' Typ der 3. vorwärts Bedingung
  lGuiWertVor(GUI_VW_VAL_3) = VAL_VORW_BED_3      ' Wert der 3. vorwärts Bedingung
  ' bg019 max 5 Kriterien
  lGuiAndOrVor(GUI_VW_AO_34) = AND_OR_VORW_34     ' Verknüpfung And/Or vorwärts Bedingungen 3 und 4
  lGuiTypVor(GUI_VW_BED_4) = TYP_VORW_BED_4       ' Typ der 4. vorwärts Bedingung
  lGuiWertVor(GUI_VW_VAL_4) = VAL_VORW_BED_4      ' Wert der 4. vorwärts Bedingung
  lGuiAndOrVor(GUI_VW_AO_45) = AND_OR_VORW_45     ' Verknüpfung And/Or vorwärts Bedingungen 4 und 5
  lGuiTypVor(GUI_VW_BED_5) = TYP_VORW_BED_5       ' Typ der 5. vorwärts Bedingung
  lGuiWertVor(GUI_VW_VAL_5) = VAL_VORW_BED_5      ' Wert der 5. vorwärts Bedingung
  
  
End Sub                           ' Ende InitPosPropArrays



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Wert zu einer Prop_ID aus Datenkapsel holen und ins entsprechende Control im GUI schreiben                    '
'                                                                                                               '
' lProp             : Property ID der Eigenschaft, deren Wert im GUI refreshed werden soll                      '
' Rückgabe          : Wert konnte angezeigt werden (True) / nicht angezeigt werden (False)                      '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
' Hier werden die effektiven Controls an die Properties gebunden, und zwar immer schön über den Umweg der..
'..GUI - Parameter Arrays. Dies hat den Vorteil, dass wenn sich die Positionen von Controls innerhalb ihres..
'..Feldes verschieben (zum Beispiel weil ein neues Control zwischendrin dazukommt), müssen nur die Konstanten..
'..im entsprechenden Array angepasst werden, der Rest stimmt dann wieder


Private Function WriteValToGUI(lProp As Long) As Boolean
  
Dim bEnableDisable As Boolean                       ' aktiv/nicht aktiv für Controls
Dim yAux As Byte                                    ' Hilfsvar
Dim iCount As Integer                               ' Schleifenzähler
Dim iAux As Integer                                 ' Hilfsvar
Dim lLocProp As Long                                ' lokale Kopie der Property
Dim lLocVal As Long                                 ' lokaler Wert


WriteValToGUI = False                               ' kein entsprechendes Control gefunden

' die verschiedenen "Regionen" der GUI nach der verlangten Prop_ID durchsuchen

' --- zuerst die sog. "Allgemeinen Einstellungen", d.h. alle Geräte - Eigenschaften, die nichts mit Kennlinien - Abschnitten zu tun haben

If (lProp <= LAD_ZEIT_MER_ALS) Then                 ' muss eine "Allgemeine Einstellung" sein
  
  If (lProp = lGuiParRedArt) Then                   ' spez: Temp Reduktions - Art (Combo - Box)
    Reduktions_art.ListIndex = FnLimit(GetGuiValFromCapsule(lGuiParRedArt), C_DERATING, C_HYSTERESE)
    WriteValToGUI = True                            ' Wert in GUI aufgefrischt
  Else                                              ' sonst: durchnummeriertes Feld gleichartiger CWNumEdits
    If (lProp = lGuiParBetrArt) Then                ' spez: Betriebs - Art Master / Booster (Combo - Box)
      ComboBetriebsart.ListIndex = FnLimit(GetGuiValFromCapsule(lGuiParBetrArt), C_STANDARD, C_CAN)
      WriteValToGUI = True                          ' Wert in GUI aufgefrischt
    Else                                            ' spez: Checkboxen für Temp.Sensoren
      If ((lProp = lGuiParAktTempSens(GUI_AKT_TMP_1)) Or (lProp = lGuiParAktTempSens(GUI_AKT_TMP_2)) Or (lProp = lGuiParAktTempSens(GUI_AKT_TMP_3))) Then
        For iCount = GUI_AKT_TMP_1 To GUI_AKT_TMP_3
          If (lProp = lGuiParAktTempSens(iCount)) Then  ' einzelne Ziffern aus Wert (z.B. 100) ausfieseln, wie "LED"
            lLocVal = GetGuiValFromCapsule(lGuiParAktTempSens(iCount))
            CheckTempSens(iCount).Value = FnLimit(FnGetCharOfNumber(lLocVal, (iCount)), C_OFF, C_ON)           ' Wert bestimmen
            WriteValToGUI = True                    ' Wert in GUI aufgefrischt
          End If
        Next iCount
      Else                                          ' sonst: durchnummeriertes Feld gleichartiger CWNumEdits
        For iCount = GUI_BATT_NENN To GUI_BOOST_MAX
          If (lProp = lGuiParArrayAllg(iCount)) Then
            CWNumEdit1(iCount).Value = GetGuiValFromCapsule(lGuiParArrayAllg(iCount))
            WriteValToGUI = True                    ' Wert in GUI aufgefrischt
          End If
        Next iCount
      End If
    End If
  End If


' --- nicht "Allgemeine Einstellungen", also Wert aus Abschnitt der Ladekennlinie

Else                                                      ' liegt die Prop innerhalb des angezeigten Abschnitts?
  If ((lProp >= (OFFSET_ABSCHN + (LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1)))) And (lProp < (OFFSET_ABSCHN + (LAENGE_ABSCHN * (g_iAktuellerAbschnitt))))) Then
    lLocProp = lProp - (LAENGE_ABSCHN * (g_iAktuellerAbschnitt - 1)) ' Abschnitts - Offset abzählen, bequemer zum Vergleichen
    
    If (lLocProp <= ANZ_RUECKW_BED) Then                    ' Anzahl Bedingungen, keine direkte GUI-Entsprechung
      
      lLocVal = GetGuiValFromCapsule(lProp)                 ' Anzahl holen
      
      If (lLocProp = ANZ_VORWAE_BED) Then                   ' Vorwärtsbedingungen
        
        For iCount = 1 To ANZ_WECHSEL_BEDINGUNGEN           ' max mögliche Anzahl durchzählen
          
          If (iCount <= lLocVal) Then                       ' Par zu Control vorhanden? Wenn ja, dann..
            BedVorwText(iCount).Visible = True              '..Control für "Art der Bedingung" anzeigen..
            BedVorwWert(iCount).Visible = True              '..Control für "Wert der Bedingung" anzeigen..
          Else                                              '..sonst..
            BedVorwText(iCount).Visible = False             '..Control für "Art der Bedingung" verbergen..
            BedVorwWert(iCount).Visible = False             '..Control für "Wert der Bedingung" verbergen
          End If
          
          If (iCount < ANZ_WECHSEL_BEDINGUNGEN) Then        ' 1 control weniger als bei Bedingungen
            If (iCount < lLocVal) Then
              AndOrButton(iCount).Visible = True            '..And/Or Knopf zeigen
            Else
              AndOrButton(iCount).Visible = False           '..And/Or Knopf verbergen
            End If
          End If
        Next iCount

      Else                                                  ' Rückwärtsbedingungen
      End If
    
    Else                                                    ' nicht Anzahl Wechselbedingungen, sondern..
      If (lLocProp <= lGuiGenAbEdBox(GUI_U_PH_UEB)) Then    '..Wert der U/I-Phasen-Edits..
        For iCount = GUI_I_PH_MAX To GUI_U_PH_UEB           '..also U/I-Edit-Böxli durchzählen
          If (lLocProp = lGuiGenAbEdBox(iCount)) Then       ' Achtung: Vergleich auf Prop OHNE Abschnitts - Offset, aber..
            num1(iCount).Value = GetGuiValFromCapsule(lProp) '..Wert holen von Prop MIT Offset
            WriteValToGUI = True                            ' Wert in GUI aufgefrischt
          End If
        Next iCount
      Else                                                        ' nicht U/I-Phasen-Edits, sondern..
        If (lLocProp <= lGuiStatusLeds(GUI_U_PH_STL)) Then        '..Status-Leds
          For iCount = GUI_I_PH_STR To GUI_U_PH_STL               '..also Status-Leds durchzählen
            If (lLocProp = lGuiStatusLeds(iCount)) Then           ' Achtung: Vergleich auf Prop OHNE Abschnitts - Offset, aber..
              lLocVal = GetGuiValFromCapsule(lProp)               '..Wert holen von Prop MIT Offset
                
              If (iCount < GUI_U_PH_STR) Then                     ' Led - Block I oder U ?
                iAux = iCount - 1                                 ' linker Block (I)
              Else
                iAux = iCount - GUI_U_PH_STR                      ' rechter Block (U)
              End If
              
              yAux = FnGetCharOfNumber(lLocVal, (iAux))           ' Wert bestimmen
              
              Select Case yAux
                
                Case LED_AUS                                      ' Aus
                  LEDButton(iCount).Value = False                 ' "dunkle LED" anzeigen
                  LEDButton(iCount).Tag = NOT_BLINKING
                
                Case LED_EIN                                      ' Ein
                  LEDButton(iCount).Value = True                  ' "helle LED" anzeigen
                  'LEDButton(iCount).OnImages("Image").BlinkInterval = cwSpeedOff
                  LEDButton(iCount).Tag = NOT_BLINKING
                  
                Case Else
                  LEDButton(iCount).Value = True                  ' Blinken
                  'LEDButton(iCount).OnImages("Image").BlinkInterval = cwSpeedFast
                  LEDButton(iCount).Tag = BLINKING                ' "blinkende LED" anzeigen
  
              End Select
              
              WriteValToGUI = True                                ' Wert in GUI aufgefrischt
            End If
          Next iCount
        Else                                                      ' nicht Status-Leds, sondern..
          If (lLocProp <= lGuiRetCheckBox(GUI_U_PH_S12)) Then     ' Checkböxli für Rückkehr zu Abschnitt 1
            For iCount = GUI_U_PH_AEG To GUI_U_PH_S12             '..also Checkböxli durchzählen
              If (lLocProp = lGuiRetCheckBox(iCount)) Then        ' Achtung: Vergleich auf Prop OHNE Abschnitts - Offset, aber..
                Check1(iCount).Value = GetGuiValFromCapsule(lProp) '..Wert holen von Prop MIT Offset
                WriteValToGUI = True                              ' Wert in GUI aufgefrischt
              End If
            Next iCount
          Else                                                    ' nicht checkböxli, sondern..
            ' bg019 max 5 Kriterien
            If (lLocProp <= lGuiWertVor(GUI_VW_VAL_5)) Then       '..Abschnittswechsel - Bedingung (Typ/Wert/Verknüpfung)
              ' Hier haben wir es mit 3 Kategorien Controls zu tun, also werden die 3 auch separat behandelt
              For iCount = GUI_VW_BED_1 To GUI_VW_BED_5           ' Alle Bedingungs-Typen durchzählen
                If (lLocProp = lGuiTypVor(iCount)) Then           ' Achtung: Vergleich auf Prop OHNE Abschnitts - Offset, aber..
                  lLocVal = GetGuiValFromCapsule(lProp)           ' Typ von Bedingung MIT Offset holen
                  BedVorwText(iCount).Caption = GetTextOfCondID(lLocVal)  ' Text zu Bedingung holen
                  WriteValToGUI = True                            ' Wert in GUI aufgefrischt
                End If
              Next iCount
              
              For iCount = GUI_VW_VAL_1 To GUI_VW_VAL_5           ' Alle Bedingungs - Werte durchzählen
                If (lLocProp = lGuiWertVor(iCount)) Then          ' Achtung: Vergleich auf Prop OHNE Abschnitts - Offset, aber..
                  BedVorwWert(iCount).Value = GetGuiValFromCapsule(lProp) ' Wert von Bedingung MIT Offset holen
                  WriteValToGUI = True                            ' Wert in GUI aufgefrischt
                End If
              Next iCount
              
              For iCount = GUI_VW_AO_12 To GUI_VW_AO_45           ' Alle Verknüpfungen durchzählen
                If (lLocProp = lGuiAndOrVor(iCount)) Then         ' Achtung: Vergleich auf Prop OHNE Abschnitts - Offset, aber..
                  AndOrButton(iCount).Value = GetGuiValFromCapsule(lProp) ' Wert von Bedingung MIT Offset holen
                  WriteValToGUI = True                            ' Wert in GUI aufgefrischt
                End If
              Next iCount

            End If                                                ' Ende "Abschnittswechsel"
          End If                                                  ' Ende "checkböxli"
        End If                                                    ' Ende "wenn Status-Leds"
      End If                                                ' Ende "wenn U/I-Phase"
    End If                                          ' Ende "wenn Anzahl wechselbedingungen"
  End If                                      ' Ende "Prop innerhalb angezeigter Abschnitt?"
End If                                        ' Ende "verschiedene Regionen"

  
End Function                      ' Ende WriteValToGUI



'////////////////////////////////// Ende GUI <-> Datenkapsel /////////////////////////////////////////




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinien - Fenster laden / initialisieren                                                                   '
' Unter anderem werden hier alle Bezeichnungen, Menü-Einträge etc. auf die richtige Sprache angepasst           '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_Load()
    
  Dim strTemp As String
  Dim strAux As String
  Dim iCount As Integer
  Dim strFileNameAndPath As String              ' Datei Name und Pfad
    

' ---------------- Internationalisierung / Localization, d.h. Texte, Menü-Einträge etc. mit korrekten Strings füllen

' ---- die Menü - Struktur ----

    Profile_Mnu.Caption = LoadResString(STR_ID_MNU_PROFILE + g_lLocaleOffset)         ' Menü "Kennlinie"
    new_profile.Caption = LoadResString(STR_ID_MNU_PROF_NEW + g_lLocaleOffset)            ' "Neu"
    load_profile.Caption = LoadResString(STR_ID_MNU_PROF_LOAD_FILE + g_lLocaleOffset)     ' "laden v. Datei"
    save_profile.Caption = LoadResString(STR_ID_MNU_PROF_SAVE_FILE + g_lLocaleOffset)     ' "speichere Datei"
    save_as.Caption = LoadResString(STR_ID_MNU_PROF_SAVE_NEW_FILE + g_lLocaleOffset)      ' "speichern unter"
    Profile_from.Caption = LoadResString(STR_ID_MNU_PROF_RECEIVE_NLG + g_lLocaleOffset)   ' "von NLG empfangen"
    Profile_to.Caption = LoadResString(STR_ID_MNU_PROF_SEND_NLG + g_lLocaleOffset)        ' "zum NLG senden"
    Quitter.Caption = LoadResString(STR_ID_MNU_PROF_EXIT_PROG + g_lLocaleOffset)          ' "Programm beenden"
    
    conn_menu.Caption = LoadResString(STR_ID_MNU_CONNECT + g_lLocaleOffset)           ' Menü "Verbindung"
    connect_nlg.Caption = LoadResString(STR_ID_MNU_CONN_NLG + g_lLocaleOffset)            ' "verbinden"
    disconnect_nlg.Caption = LoadResString(STR_ID_MNU_DISCONN_NLG + g_lLocaleOffset)      ' "trennen"
    
    extra1.Caption = LoadResString(STR_ID_MNU_EXTRAS + g_lLocaleOffset)               ' Menü "Extras"
    info_nlg.Caption = LoadResString(STR_ID_MNU_EXTRA_CHG_INFO + g_lLocaleOffset)         ' "NLG Info"
    Extra_Options.Caption = LoadResString(STR_ID_MNU_EXTRA_PROPERTIES + g_lLocaleOffset)  ' "Eigenschaften"
    show_comment.Caption = LoadResString(STR_ID_MNU_EXTRA_OPTIONS + g_lLocaleOffset)      ' "Optionen"
    
    Help.Caption = LoadResString(STR_ID_MNU_HELP + g_lLocaleOffset)                   ' Menü "Hilfe"
    HelpF1.Caption = LoadResString(STR_ID_MNU_HLP_CHGST_HLP + g_lLocaleOffset)            ' "Hilfe F1"
    HelpContents.Caption = LoadResString(STR_ID_MNU_HLP_CONT + g_lLocaleOffset)           ' "Inhalt"
    HelpAbout.Caption = LoadResString(STR_ID_MNU_HLP_ABOUT + g_lLocaleOffset)             ' "über ChargeStar"


' ---- Allgemeines ----
    
    strAux = LoadResString(STR_ID_APP_TITLE + g_lLocaleOffset)             ' Titel der Applikation festlegen
    kennlinie.Caption = CompressTextToMaxLength(strAux, kennlinie.Width, kennlinie)   ' bg018
    
    status_fenster(0).Caption = LoadResString(STR_ID_RTX_NOT_CONN + g_lLocaleOffset)      ' "nicht verbunden"
    status_fenster(1).Caption = LoadResString(STR_ID_RTX_CONNECTED + g_lLocaleOffset)     ' "verbunden"
    LblMitGeraet.Caption = LoadResString(STR_ID_RTX_UNIT + g_lLocaleOffset)               ' "mit Gerät"

' Allgemeine Einstellungen, Texte der Controls
    LabelAllg.Caption = LoadResString(STR_ID_GENERAL_SETTINGS + g_lLocaleOffset)      ' "Allgemeine Einstellungen"
    TextUnenn.Text = LoadResString(STR_ID_GEN_NOM_BAT_VOLT + g_lLocaleOffset)             ' "Nennspannung"
    TextTempKoeff.Text = LoadResString(STR_ID_GEN_TEMP_KOEF_GAS + g_lLocaleOffset)        ' "Temp.Koeffizient Gasungsspannung"
    TextRefTemp.Text = LoadResString(STR_ID_GEN_REF_TEMP_VOLT + g_lLocaleOffset)          ' "BezugsTemp. Spgs-Angaben"
    TextTempSensoren.Text = LoadResString(STR_ID_GEN_ACT_TEMP_SENS + g_lLocaleOffset)     ' "aktive Temp.sensoren"
    TextMaxINetz.Text = LoadResString(STR_ID_GEN_MAX_MAINS_CURR + g_lLocaleOffset)        ' "max. Netzstrom"
    TextTempRedArt.Text = LoadResString(STR_ID_GEN_TEMP_RED_MODE + g_lLocaleOffset)       ' "Temp. Reduktionsart"
    Reduktions_art.List(0) = LoadResString(STR_ID_GEN_MODE_DERATING + g_lLocaleOffset)    ' Listen-Eintrag "Derating"
    Reduktions_art.List(1) = LoadResString(STR_ID_GEN_MODE_HYSTERESIS + g_lLocaleOffset)  ' Listen-Eintrag "Hysterese"
    TextBatTempDerat.Text = LoadResString(STR_ID_GEN_BAT_TEMP_DERAT + g_lLocaleOffset)    ' "Temp. Derating"
    TextBezTempDerat.Text = LoadResString(STR_ID_GEN_REF_DERAT_ZERO + g_lLocaleOffset)    ' "Derating to Zero"
    ' Text-Zuweisungen für Hysterese bei der Umschaltfunktion  Reduktions_art_Click()

' Betriebsart
    LabelBetrArt.Caption = LoadResString(STR_ID_OPERATION_MODE + g_lLocaleOffset)     ' "Betriebsart"
    TextBetriebsart.Text = LoadResString(STR_ID_OP_NLG_OP_MODE + g_lLocaleOffset)         ' "NLG Betriebsart"
    ComboBetriebsart.List(0) = LoadResString(STR_ID_OP_AUTO + g_lLocaleOffset)            ' Listen-Eintrag "Auto"
    ComboBetriebsart.List(1) = LoadResString(STR_ID_OP_BOOSTER + g_lLocaleOffset)         ' Listen-Eintrag "Booster"
    ComboBetriebsart.List(2) = LoadResString(STR_ID_OP_CAN + g_lLocaleOffset)             ' Listen-Eintrag "CAN"
    TextMinIBoost.Text = LoadResString(STR_ID_OP_CURR_BOOST_OFF + g_lLocaleOffset)        ' "Strom Booster Aus"
    TextMaxIBoost.Text = LoadResString(STR_ID_OP_CURR_BOOST_FULL + g_lLocaleOffset)       ' "Strom Booster Vollleistung"

' Sicherheits-Abschaltungen
    LabelSicherAbschalt.Caption = LoadResString(STR_ID_SAFETY + g_lLocaleOffset)      ' "Abschaltung in jedem Fall"
    TextTempUeb.Text = LoadResString(STR_ID_SAFETY_BAT_TEMP + g_lLocaleOffset)            ' "Bat. Temp. über"
    TextSpgUeb.Text = LoadResString(STR_ID_SAFETY_BAT_VOLT + g_lLocaleOffset)             ' "Bat. Spannung über"
    TextLadUeb.Text = LoadResString(STR_ID_SAFETY_CHARGE_AH + g_lLocaleOffset)            ' "Bat. Ladung über"
    TextZeitUeb.Text = LoadResString(STR_ID_SAFETY_TIMEOUT + g_lLocaleOffset)             ' "Ladezeit über"

' Lade - Profil
    LabelLadeKennl.Caption = LoadResString(STR_ID_CHARGING_PROFILE + g_lLocaleOffset) ' "Ladekennlinie"
    AbschnZurueck.Caption = LoadResString(STR_ID_PRF_PREVIOUS + g_lLocaleOffset)          ' Button "Zurück"
    ' ComboAbschnitt wird unten eingefüllt
    AbschnVor.Caption = LoadResString(STR_ID_PRF_NEXT + g_lLocaleOffset)                  ' Button "Vor"
    TextMaxStrom.Text = LoadResString(STR_ID_PRF_MAX_CHG_CURR + g_lLocaleOffset)          ' "Max. Ladestrom"
    TextMaxSpg.Text = LoadResString(STR_ID_PRF_MAX_CHG_VOLT + g_lLocaleOffset)            ' "Max. Ladespannung"

' Bedingungen für Lade - Profil
    LabelWechselBed.Caption = LoadResString(STR_ID_PRF_NEXT_SECT_IF + g_lLocaleOffset) ' "Abschnitt wechseln wenn.."
    For iCount = 1 To (ANZ_WECHSEL_BEDINGUNGEN - 1)                                       ' And/Or Buttons beschriften
      AndOrButton(iCount).OffText = LoadResString(STR_ID_PRF_OR + g_lLocaleOffset)        ' ODER
      AndOrButton(iCount).OnText = LoadResString(STR_ID_PRF_AND + g_lLocaleOffset)        ' UND
    Next iCount

' ToolTipTexte für Bedingungen setzen / löschen
    BedingTypVW.ToolTipText = LoadResString(STR_ID_HLP_ADD_COND + g_lLocaleOffset)        ' doppelklick zum hinzufügen
    For iCount = 1 To ANZ_WECHSEL_BEDINGUNGEN
      BedVorwText(iCount).ToolTipText = LoadResString(STR_ID_HLP_REMOVE_COND + g_lLocaleOffset) ' doppelklick zum entfernen
    Next iCount
    
    
' ---------- globale Variablen, die initialisiert werden müssen

    'alt, vermutlich unnötig'  App.HelpFile = App.Path & "\Help_File\ChargeStarHelp.hlp"
    
    If (g_lLocaleOffset = LOCALE_DEUTSCH) Then
      App.HelpFile = App.Path & "\ChargeStarHelp_D.hlp"
    Else
      App.HelpFile = App.Path & "\ChargeStarHelp_E.hlp"
    End If
      
    datei_fenster1.InitDir = CurDir()   ' Fokus auf aktuelles Arbeitsverzeichnis setzen
    
    Call InitPosPropArrays              ' Zuordnungen von Controls zu Prop_ID's der Parameter herstellen

    sCurrentHeight = NORMAL_HEIGHT      ' Fenster auf Normalhöhe bringen

    BlinkTimer.Interval = BLINK_INTERVALL   ' allgemeine Blinkfrequenz zuweisen
    
    g_bConnectionEstablished = False    ' zunächst ausgeloggt

    g_strCommentText = ""               ' Kommentar - Text ist zunächst leer




' ---------- Menü - Einträge aktivieren / deaktivieren

    connect_nlg.Enabled = True          '..Verbindung herstellen möglich und..
    disconnect_nlg.Enabled = False      '..Verbindung abbrechen nicht möglich und..
    Profile_from.Enabled = False        '..Kennline vom Gerät laden nicht möglich und..
    Profile_to.Enabled = False          '..Kennlinie zum Gerät schicken nicht möglich und..
    info_nlg.Enabled = False            '..Info abfragen nicht möglich und..
    status_fenster(1).Visible = False
    status_fenster(0).Visible = True    ' zeige "not connected" an
    


' ---------- GUI Controls anpassen / updaten

    ' anyway Beta/Debug
    If (SWITCH_DEBUG = False) Then      ' keine Debug - Version, also Label "Debug" verbergen
      LblDebug.Visible = False
    End If


    ' Abschnitt - Combobox mit "Abschnitt 1", "Abschnitt 2"  usw. füllen
    For iCount = 1 To ANZ_LADEABSCHNITTE
      strTemp = LoadResString(STR_ID_PRF_SECTION + g_lLocaleOffset) + Str(iCount)
      ComboAbschnitt.AddItem strTemp, iCount - 1
    Next iCount
    
    ComboAbschnitt.ListIndex = 0        ' beginne mit Abschnitt 1
    
    ' Vorwärts - Bedingungs - Combobox mit den vorhandenen Bedingungen füllen
    iCount = 0
    While ((iCount < MAX_COND) And (stCond(iCount).iCondID < END_PAR))
      BedingTypVW.AddItem stCond(iCount).strCondTxt                           ' neuen Text dazu
      BedingTypVW.ItemData(BedingTypVW.NewIndex) = stCond(iCount).iCondID     ' neue ID dazu
      iCount = iCount + 1
    Wend
    
    ' defaultigste Default-Kennlinie laden (default.ini), sonst "hart codierte" defaults
    strFileNameAndPath = App.Path & "\default.ini"                        'Default Datei
    
    If (GetProfileFromFile(strFileNameAndPath, False) = STATE_OK) Then    ' vom Ini-File laden
    Else
      Call InitCapsuleToDefault                 ' "hart" codierte Defaults laden
    End If

    
    Call UpdateGUI(UPDATE_ALL)                            ' alles anzeigen
    
    g_bProfileChanged = False           ' Bis jetzt nichts händisch verändert, Kennlinie noch nicht "speicherwürdig"

End Sub                                                   ' Ende Form_Load



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fenster - Grösse ändern, gegen oben beschränkt                                                                 '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_Resize()

  If WindowState = vbMaximized Then
    WindowState = vbNormal
    kennlinie.Width = MAX_WIDTH
    kennlinie.Height = sCurrentHeight
  End If
  
  If (kennlinie.Width >= MAX_WIDTH) Then
    kennlinie.Width = MAX_WIDTH
  End If
  
  If (kennlinie.Height >= sCurrentHeight) Then
    kennlinie.Height = sCurrentHeight
  End If

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' ChargeStar Haupt - Fenster abschiessen; gibt eine begrenzte Einflussmöglichkeit auf die Windös - ..           '
'..Abschussmechanismen                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_QueryUnload(Cancel As Integer, UnloadMode As Integer)
    
Dim bEnableQuit As Boolean                      ' Flag für "beenden erlaubt"

  bEnableQuit = False
  
  If g_bProfileChanged = True Then              ' schon was "speicherwürdiges" vorhanden?
    If (AskForSaveProfile = True) Then          ' Abgang freigeben?
      bEnableQuit = True                        ' ja, Abgang geht klar
    End If
  Else                                          ' nöö, nix zu speichern
    bEnableQuit = True                          ' ja, Abgang geht klar
  End If

  If (FrmSpgKonvert.Visible = True) Then        ' das Nennspannungs - Konverter-Fensterli schwirrt noch herum
    FrmSpgKonvert.WindowState = vbNormal        ' Sonst setzen wir's auf Normalgrösse und..
    FrmSpgKonvert.ZOrder                        '..bringen's in den Vordergrund
    bEnableQuit = False
  End If

  If (bEnableQuit = False) Then                 ' Nöö, kein Abgang erlaubt !
    Cancel = True
  Else                                          ' bg009 wenn Abgang erlaubt, dann mach wenigstens..
    Call DisconnectNLG                          '..noch einen geordneten Logout, falls mit NLG connected
  End If
  
End Sub                                         ' Ende Form_QueryUnload





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Blinkende Controls (wie z.B. LEDs im Zustand "blinkend") auffrischen                          '
'                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub BlinkTimer_Timer()

Dim iCount As Integer

  bBlinkState = Not bBlinkState               ' Blinker toggeln
  
  For iCount = GUI_I_PH_STR To GUI_U_PH_STL   ' alle LEDs durchzählen
    If LEDButton(iCount).Tag = BLINKING Then  ' sind die Dinger als "blinkend" markiert?
      LEDButton(iCount).Value = bBlinkState   ' wenn ja, aktuellen Blinkzustand (Ein/Aus) zuweisen
    End If
  Next iCount
  
End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fehlermeldung oder Hinweis anzeigen                                                                           '
'                                                                                                               '
' lMsgNbr       : Nummer der Meldung                                                                            '
' strAddText    : String mit zusätzlichem Text für die Meldung
' Return        : Rückgabewert der Dialogbox, i.d.R. "OK" oder "Cancel"                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function DisplayErrOrMessage(ByVal lMsgNbr As Long, ByVal strAddText As String) As Long
    
  DisplayErrOrMessage = vbOK
 
  Select Case lMsgNbr
      
    
    '--------------------- Datei - Fehler -------------
    
    Case ERROR_FILE_SAVE
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_FILE_CANT_SAVE + g_lLocaleOffset))
    
    Case ERROR_FILE_LOAD
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_FILE_CANT_LOAD + g_lLocaleOffset))
    
    Case ERROR_FILE_FORMAT
      DisplayErrOrMessage = MsgBox("'" & strAddText & LoadResString(STR_ID_ERR_FILE_NOT_VALID + g_lLocaleOffset))
    
    Case USER_FILE_QUIT

    
    '--------------------- Kommunikations - Fehler -------------------
   
    Case CONNECT_ERROR
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_CANT_CONNECT + g_lLocaleOffset), vbRetryCancel)
     
    Case FRAME_RECEIVE_ERROR
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_PROPERTY__ + g_lLocaleOffset) & strAddText & LoadResString(STR_ID_ERR__CANT_RECEIVE + g_lLocaleOffset), vbAbortRetryIgnore)
     
    Case FRAME_SEND_ERROR
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_PROPERTY__ + g_lLocaleOffset) & strAddText & LoadResString(STR_ID_ERR__CANT_SEND + g_lLocaleOffset), vbAbortRetryIgnore)
     
    Case CHARGE_STOP_ERROR        ' Grund: Message "Stop Ladung" nicht quittiert -> NLG und PC : AEG
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_CANT_SEND_RESTART + g_lLocaleOffset))
     
    Case CHARGE_START_ERROR       ' Grund: Message "Start Ladung" nicht quittiert -> NLG und PC : AEG
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_RESTART_ACTIV_PROF + g_lLocaleOffset))
      
    Case comPortAlreadyOpen       ' VB - Konstante
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_SER_PORT_IN_USE + g_lLocaleOffset))
      
    
    '--------------------- Sicherheits - Abfragen / Meldungen -------------
    
    Case PROFILE_DATA_CHANGE      ' wollen speichern?
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_TXT_SAVE_CURR_PROF + g_lLocaleOffset), vbYesNo)
      
    Case NO_CONNECTION_TO_NLG
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_TXT_NO_CONN_NLG + g_lLocaleOffset))
      
    Case OLD_FIRMWARE
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_TXT_OLD_FW_NLG + g_lLocaleOffset))
    
    
    '--------------------- logische Fehler -------------
    Case ALL_CONDITIONS_USED      ' max. Anzahl Bedingungen für Kennlinien-Umschaltung erreicht
      DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_NO_MORE_COND_ALLOWED + g_lLocaleOffset))
      
    ' --------------------- sonstige Fehler / Meldungen ----------------------------
    Case Else
      If (lMsgNbr < 1000) Then  ' Fehler wird von VB/Windows abgehandelt; theoretisch sollte das auch mit den Nummern ab 31000 so sein,..
        DisplayErrOrMessage = MsgBox(Error(lMsgNbr))  '..aber da gibt's nur die Wischi-Waschi-Meldung "Anwendungs- oder objektdefinierter Fehler"
      Else                      ' "merkwürdiger" Fehler der seriellen Schnittstelle (nicht vorhanden, falsch konfiguriert oder so)
        If ((lMsgNbr >= comPortOpen) And (lMsgNbr <= comDCBError)) Then
          DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_SER_PORT_NOT_READY + g_lLocaleOffset))
        Else                    ' Fehler wird von der Applikation angezeigt, damit man wenigstens die Nummer hat
          DisplayErrOrMessage = MsgBox(LoadResString(STR_ID_ERR_UNKNOWN_ERR__ + g_lLocaleOffset) & lMsgNbr & LoadResString(STR_ID_ERR__OCCURRED + g_lLocaleOffset))
        End If
      End If
  
  End Select
    
End Function                      ' Ende DisplayErrOrMessage



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Datei Dialog (laden / speichern) anzeigen, Datei Name und Pfad ermitteln und auf Fehler überprüfen; dazu..    '
'..wird das CommonDialog Control benutzt, welches gewisse Fehlermeldungen und Rückfragen selber abhandeln kann  '
'                                                                                                               '
' bSave         : Datei speichern (True) oder laden (False)                                                     '
' bValid        : gültiger Dateiname gewählt(True) oder Abbruch o.ä.                                            '
' Return        : Name der zu ladenden oder speichernden Datei                                                  '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function GetFileName(ByVal bSave As Boolean, ByRef lErrMsg As Long) As String
     
Dim objFileTemp As Object                   ' temporäres File System Objekt
Dim bExists As Boolean                      ' File existiert / existiert nicht
Dim iAux As Integer


lErrMsg = NO_ERROR                          ' Datei - Name und Pfad gültig
    
On Error GoTo LABEL_FEHLER                  ' "Error Handler" Abschnitt, naja...

  ' N.B.: "datei_fenster1" ist als (unsichtbares) Control auf dem Kennlinien-Formular enthalten. Es dient als eigenständiges,..
  '..aktivierbares Objekt, um Standard - Dialoge wie Datei-Manager, Hilfe-System etc anzuzeigen
'  datei_fenster1.DefaultExt = "Kennlinien (*.pfl)|*.pfl"    ' Default - Endung, wenn nichts angegeben
'  datei_fenster1.Filter = "Alle Dateien (*.*)|*.*|" & "Kennlinien (*.pfl)|*.pfl"  ' Endungen *.pfl und *.* stehen zur Auswahl
'  datei_fenster1.FilterIndex = 2                            ' per Default werden *.pfl Dateien vorausgesetzt

  datei_fenster1.DefaultExt = LoadResString(STR_ID_TXT_PROF_PFL + g_lLocaleOffset)    ' Default - Endung, wenn nichts angegeben
  datei_fenster1.Filter = LoadResString(STR_ID_TXT_ALL_FILES + g_lLocaleOffset) & LoadResString(STR_ID_TXT_PROF_PFL + g_lLocaleOffset)  ' Endungen *.pfl und *.* stehen zur Auswahl
  datei_fenster1.FilterIndex = GUI_DEF_FILE_EXT_IDX                            ' per Default werden *.pfl Dateien vorausgesetzt

  
  If (bSave = True) Then
    datei_fenster1.DialogTitle = "Datei speichern"
    ' kein Schreibschutz, bei existierender Datei "überschreiben?" abfragen
    datei_fenster1.Flags = cdlOFNHideReadOnly + cdlOFNOverwritePrompt
    datei_fenster1.ShowSave
  Else
    datei_fenster1.DialogTitle = "Datei laden"
    ' kein Schreibschutz, Datei muss existieren
    datei_fenster1.Flags = cdlOFNHideReadOnly + cdlOFNFileMustExist
    datei_fenster1.ShowOpen
  End If
    
  GetFileName = datei_fenster1.FileName       ' gewünschte Datei, Name und Pfad
  
  
  Exit Function                               ' nix Fehler, also aussteigen


LABEL_FEHLER:                                 ' "Error Handler" Abschnitt
        
  lErrMsg = Err.Number
  
  GetFileName = ""

Exit Function

End Function                  ' Ende GetFileName





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Abfragen, ob die aktuelle Kennlinie gespeichert werden soll, ggf. Speicherung vornehmen. Die Entscheidung,..  '
'..wann eine solche Abfrage gemacht werden soll, liegt beim Aufrufer                                            '
'                                                                                                               '
' Rückgabe          : neuer Datensatz darf geladen werden J/N                                                   '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function AskForSaveProfile() As Boolean
  
  Dim strFileNameAndPath As String              ' Datei Name und Pfad
  Dim strAux As String                            ' Hilfs-String
  Dim lErrOk As Long                            ' Error ID
  Dim lRetVal As Long                           ' Rückgabe - Wert der Message Box
  
  
  lRetVal = DisplayErrOrMessage(PROFILE_DATA_CHANGE, vbYesNo) ' wollen speichern? Antwort erfragen
  
  If (lRetVal = vbYes) Then                     ' Speicherung erwünscht
  
    strFileNameAndPath = ""                                    ' Datei - Name zunächst leer
    strFileNameAndPath = GetFileName(FILE_SAVE, lErrOk)        ' Datei - Name und Pfad ermitteln
    
    If (lErrOk = NO_ERROR) Then                 ' alles klar, erst speichern, dann neu laden
      If (PutProfileToFile(strFileNameAndPath) = True) Then       ' Speichern erfolgreich?
        strAux = LoadResString(STR_ID_APP_TITLE + g_lLocaleOffset) & " - " & strFileNameAndPath
        kennlinie.Caption = CompressTextToMaxLength(strAux, kennlinie.Width, kennlinie)   ' bg018
        AskForSaveProfile = True                ' neu laden erlaubt
      Else
        AskForSaveProfile = False               ' speichern erwünscht aber erfolglos, also nix neues laden
      End If
    Else                                        ' irgendaan Fääla
      If (lErrOk = USER_FILE_QUIT) Then         ' Speichern vom Benutzer abgebrochen, aber neu laden erlaubt
        AskForSaveProfile = True
      Else                                      ' sonschtwas, tu nix
        Call DisplayErrOrMessage(lErrOk, "")    ' Fehlermeldung anzeigen
        AskForSaveProfile = False               ' Speichern ging nicht, also nichts neues laden
      End If
    End If
  
  Else                                          ' Speicherung nicht erwünscht
    AskForSaveProfile = True                    ' neu laden erlauben
  End If

End Function





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinie aus einer Text-Datei lesen, in die Datenkapsel schreiben.                                           '
'                                                                                                               '
' strFile           : Name und Pfad der Text - Datei                                                            '
' Rückgabe          : Status der Datei (Datei-Format i.O. oder falsch)                                          '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function GetProfileFromFile(strFile As String, bShowErr As Boolean) As Integer
   
  On Error GoTo LBL_GET_PROF_ERR_HANDLER
   
  Dim strLoadFile As String                 ' zu ladende Datei
  Dim objFileSystem As Object               ' genauer: ActiveX Object
  Dim objText                               ' "TextStream" Objekt
  Dim strTemp As String                     ' temporärer String
  Dim strParts() As String                  ' Array von Strings, für die Auswertung der Zeilen
  Dim lVal As Long                          ' Long Hilfsvar
  Dim iSeg As Integer                       ' Abschnitt Nr
  Dim bNext As Boolean                      ' nächster Par vorhanden?
  Dim yHi As Byte, yLo As Byte              ' Hi / Lo Byte akt Prop
  Dim yHiPrev As Byte, yLoPrev As Byte      ' Hi / Lo Byte vorherige Prop
  Dim iRowCount As Integer                  ' Zeilen - Zähler
  Dim iInnerState As Integer                ' innerer Zustand, für Eigenschaften - String
  Dim iTmp As Integer                       ' temp zahl
  Dim strTmp2 As String                     ' noch'n temp string
  
  
  GetProfileFromFile = STATE_ERROR          ' bis jetzt als "ungültig" zu betrachten
  iSeg = 0                                  ' zunächst die "allgemeinen Einstellungen"
  iRowCount = 0                             ' Zeilen - Zähler 0
  iInnerState = STATE_IDLE
  iBackUpNLGType = 0                        ' NLG-Typ noch nicht bekannt
  
  '"File System Object", aus dem Scripting Päckchen
  Set objFileSystem = CreateObject("Scripting.FileSystemObject")
  ' Datei als objekt öffnen, nur zum lesen (1)
  Set objText = objFileSystem.OpenTextFile(strFile, 1)
  
  
  ' komplette Datei Zeile für Zeile auswerten, Begrenzung für "unendliche" Dateien
  While ((objText.AtEndOfStream <> True) And (iRowCount < 4 * MAX_PAR))
    
    iRowCount = iRowCount + 1       ' 1 Zeile weiter
    yHi = NOT_PAR
    yLo = NOT_PAR
    yHiPrev = NOT_PAR
    yLoPrev = NOT_PAR               ' bei jeder neuen Zeile alle möglichen Prop's checken, deshalb..
    bNext = True                    '..komplette Initialisierung
    
    strTemp = objText.readline      ' eine Zeile lesen
  
    If (strTemp <> "") Then         ' keine Leerzeilen auswerten
      
      If (Left(Trim(strTemp), 1) = "[") Then                                        ' Block - Bezeichner..
        strParts = Split(strTemp, " ", -1, vbTextCompare)                           '..auswerten; wenn..
        If (StrComp(Trim(strParts(0)), "[Abschnitt", vbTextCompare) = 0) Then       '.."Abschnitt " drin steht..
          iSeg = CInt(Val(Trim(strParts(1))))                                       '..Nr. des Abschnitts merken..
        Else                                                                        ' ..sonst "BRUSA" am Anfang?,..
          If ((GetProfileFromFile = STATE_ERROR) And (StrComp(Trim(UCase(strParts(0))), "[BRUSA", vbTextCompare) = 0)) Then
            GetProfileFromFile = STATE_A                                            '..ok, einen Zustand weiterschalten
          Else                                                                      '"Allgemeine" als nächstes? ok,..
            If ((GetProfileFromFile = STATE_A) And (StrComp(Trim(strParts(0)), "[Allgemeine", vbTextCompare) = 0)) Then
              GetProfileFromFile = STATE_OK                                         '.. dann Datei wahrscheinlich i.O.
            Else                                  ' Eigenschaften/Kommentar Text-Feld gefunden; alle folgenden Zeilen..
              If ((GetProfileFromFile = STATE_OK) And (StrComp(Trim(strParts(0)), "[Start", vbTextCompare) = 0)) Then
                iInnerState = STATE_A             '..gelten als Teil dieses Texts, bis ein eindeutiges Ende gefunden ist
              Else
                If ((iInnerState = STATE_A) And (StrComp(Trim(strParts(0)), "[Ende", vbTextCompare) = 0)) Then
                  iInnerState = STATE_IDLE        ' Ende des Eigenschaften-Texts gefunden
                End If
              End If
            End If
          End If
        End If
        
      Else                          ' kein Blockbezeichner, sondern ein Inhalt wie Property-Text, Kommentar-Text etc.
        
        If (iInnerState = STATE_A) Then                 ' Kommentar-Text, "Spezialfall"
          
          g_strCommentText = g_strCommentText & strTemp & vbNewLine
          
        Else                                            ' Property - Text, "Normalfall"
          
          strParts = Split(strTemp, "=", -1, vbTextCompare)                             ' Text nach Gleichheitszeichen trennen
          
          If (iSeg = 0) Then        ' "Allgemeine Einstellungen"
          
            If (iBackUpNLGType = 0) Then                                                ' bg024 NLG-Typ schon abgefragt?
              If (Left(Trim(strTemp), 5) = "NLG 5") Then                                ' NLG-Typ Eintrag, gültig?
                iBackUpNLGType = FnMax(CInt(Val(Right(Trim(strTemp), 3))), 1)           ' ganze Zahl NLG-Typ merken,
              End If
            End If
           
            While ((bNext = True) And (FnLongOf2Bytes(yHi, yLo) <= MAX_ALLG_PAR))       ' Schleife über "Allgemeine Einstellungen"
              
              If (GetTextOfProperty(yHi, yLo) = strParts(0)) Then                       ' stimmt der Text überein?
                lVal = CLng(Val(strParts(1)) * GetScaleOfPar(yHi, yLo))                 ' sauberen Wert draus machen
                Call PutDataToCapsule(yHi, yLo, lVal)                                   ' Wert in die Kapsel schreiben
              End If
        
              yHiPrev = yHi         ' nächste Prop abchecken
              yLoPrev = yLo
              bNext = GetNextPar(yHiPrev, yLoPrev, yHi, yLo)
      
            Wend
    
          Else                      ' "Ladeabschnitte"
                                    ' Schleife über alle möglichen Par bis Ende dieses Abschnitts
            While ((bNext = True) And (FnLongOf2Bytes(yHi, yLo) < (OFFSET_ABSCHN + iSeg * LAENGE_ABSCHN)))
                                    ' liegt der gesuchte Par in diesem Abschnitt?
              If ((FnLongOf2Bytes(yHi, yLo) >= (OFFSET_ABSCHN + (iSeg - 1) * LAENGE_ABSCHN)) And _
                  (FnLongOf2Bytes(yHi, yLo) < (OFFSET_ABSCHN + iSeg * LAENGE_ABSCHN))) Then
                If (GetTextOfProperty(yHi, yLo) = strParts(0)) Then                     ' stimmt der Text überein?
                  lVal = CLng(Val(strParts(1)) * GetScaleOfPar(yHi, yLo))               ' sauberen Wert draus machen
                  Call PutDataToCapsule(yHi, yLo, lVal)                                 ' Wert in die Kapsel schreiben
                End If
              End If
              
              yHiPrev = yHi         ' nächste Prop abchecken
              yLoPrev = yLo
              bNext = GetNextPar(yHiPrev, yLoPrev, yHi, yLo)
      
            Wend                    ' Ende Schleife über Par bis Ende Abschnitt
                  
          End If                    ' Ende "in Ladeabschnitten"
      
        End If                      ' Ende Property-Text
      
      End If                        ' Ende " Kommentar-Text oder Property-Text"
    
    End If                          ' Ende "nicht Leerzeile"
  
  Wend                              ' nächste Zeile der Datei auswerten
  
  lVal = 0

  Exit Function                     ' hier raus, sonst geht er durch den Error Handler auch durch !



LBL_GET_PROF_ERR_HANDLER:

  If (bShowErr = True) Then         ' Anzeige von Fehlern erwünscht?
    Call DisplayErrOrMessage(Err.Number, "")
  End If
  On Error GoTo 0
    
End Function                ' Ende GetProfileFromFile




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinie in Form von Text - Zeilen in einer Datei speichern; die Daten werden aus der Datenkapsel geholt.    '
'                                                                                                               '
' strFile           : Name und Pfad der Text - Datei                                                            '
' Return            : Funktions - Status; Funktion erfolgreich (True) oder Fehler (False)                                                                                                              '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function PutProfileToFile(strFile As String) As Boolean
  
  On Error GoTo LBL_PUT_PROFILE_ERR_HANDLER
  
  Dim strTemp As String                     ' zusammengesetzter String
  Dim strAux As String                      ' Hilfs - String
  Dim strAux2 As String                     ' noch ein Hilfs - String
  Dim iSeg As Integer                       ' Abschnitt Nr
  Dim iLoop As Integer                      ' Loop - Var
  Dim iType As Integer                      ' NLG Typ
  Dim objFileSystem As Object               ' genauer: ActiveX Object
  Dim objText                               ' "TextStream" Objekt
  Dim yHi As Byte, yLo As Byte              ' aktuelle Prop_ID
  Dim yHiPrev As Byte, yLoPrev As Byte      ' vorherige Prop_ID
  Dim bNext As Boolean                      ' nächster Par vorhanden J/N
  Dim lProp                                 ' Property als Long
  Dim lThisID As Long                       ' aktuelle Locale ID
  PutProfileToFile = True                   ' Funktion bis jetzt fehlerlos
  
  ' Achtung! die folgende Sch.. funktioniert nur, wenn der Windows Scripting Host installiert ist:
  ' Systemsteuerung/Software/WindowsSetup/Zubehör/WindowsScriptingHost ankreuzen!
  '"File System Object", aus dem Scripting Päckchen holen/erstellen
  Set objFileSystem = CreateObject("Scripting.FileSystemObject")
  ' Datei aus FileSystem-Objekt erstellen, mit der Methode CreateTextFile; ergibt ein sog. "TextStream" Objekt
  Set objText = objFileSystem.CreateTextFile(strFile, True)
  
  lThisID = GetUserDefaultLCID()        ' LocaleID (Ländereinstellung) dieses Users holen
  
  
  ' Allgemeine Daten
  objText.writeline ("[BRUSA Elektronik AG]")
  
  
  '------------------------- Read Only Info -------------------------------------
  ' Geräte Typ und Version
      
  strTemp = "NLG "                      ' NLG-Typ holen und auswerten (X von "NLG 51X")
  
  iType = NLG_MIN_KLASSE + (FnLoByteOfLong(GetDataFromCapsule(0, INF_NLG_TYP_51X)))
                                        ' bg024 sinnvollen Wert in der Datei nicht mit sinnlosem Default überschreiben
  If (iType <= NLG_MAX_KLASSE) Then     ' "potentiell mögliches" Gerät?
    If (iType = NLG_MIN_KLASSE + VAL_INF_NLG_TYP_51X) Then  ' Default-Eintrag in Datenkapsel ("519"). War in der ..
      If ((iBackUpNLGType < iType) And (iBackUpNLGType >= NLG_MIN_KLASSE)) Then    '..pfl-Datei ächt schon etwas ..
        iType = iBackUpNLGType          '..sinnvolleres drin? Dann dieses wieder reinschreiben
      End If
    End If
    strTemp = strTemp & CStr(iType)     ' NLG-Typ eintragen (aktuell: zwischen 510 und 516)
  Else
    strTemp = strTemp & "5xx"           ' "generischen NLG-Typ" eintragen
  End If
 
  
  objText.writeline (Trim(strTemp))
  
  objText.writeline ("")
  objText.writeline ("")
  
  
  
  '------------------------- Allgemeine Einstellungen -------------------------------------
  
  objText.writeline ("[Allgemeine Einstellungen]")

  yHiPrev = NOT_PAR                                   ' Am Anfang ist der "vorherige Par" der "NichtParameter" an Stelle 0
  yLoPrev = NOT_PAR                                   '
  yHi = NOT_PAR                                       '
  yLo = NOT_PAR                                       '
  bNext = True                                        ' wenigstens den Null - Parameter wird's wohl geben ;-)
  lProp = FnLongOf2Bytes(yHi, yLo)                    ' "Not_Par" Property als Long

  
  While ((bNext = True) And (lProp <= MAX_ALLG_PAR))  ' Schleife über "Allgemeine Einstellungen"
                                                        ' gültige Property mit vorhandenem Text
    If ((lProp > MAX_RDONLY) And (GetTextOfProperty(yHi, yLo) <> "")) Then
      strTemp = GetTextOfProperty(yHi, yLo)           ' Text zur Property mit zugehörigem Wert auf eine Zeile schreiben
      If (lProp <> AKTIVE_TEMP_SENS) Then             ' nicht die "aktiven Temperatur - Sensoren", also normale Darstellung
        ' neu
        strAux = Trim(CStr(GetDataFromCapsule(yHi, yLo) / GetScaleOfPar(yHi, yLo)))
        strAux = ConvertNumberToLocaleString(strAux, lThisID)   ' Zahlen immer mit Dezimalpunkt ablegen
        strTemp = strTemp & "=" & strAux
        ' ende neu
      
        ' alte version
        ' strTemp = strTemp & "=" & Trim(CStr(GetDataFromCapsule(yHi, yLo) / GetScaleOfPar(yHi, yLo)))
        ' ende alt
      
      Else                                            ' die drei TempSensoren, immer dreistellig anzeigen
        strAux = Trim(CStr(GetDataFromCapsule(yHi, yLo) / GetScaleOfPar(yHi, yLo)))
        strAux = Format(strAux, "000")
        strTemp = strTemp & "=" & strAux
      End If
      objText.writeline (strTemp)
    End If
    
    yHiPrev = yHi
    yLoPrev = yLo
    bNext = GetNextPar(yHiPrev, yLoPrev, yHi, yLo)    ' nächste Property holen
    lProp = FnLongOf2Bytes(yHi, yLo)                  ' Property als Long
  
  Wend

  
  '----------------------- die 7 Ladeabschnitte schreiben -------------------------------------
  
  For iSeg = 1 To ANZ_LADEABSCHNITTE                  ' Schleife über diesen Abschnitt
  
    objText.writeline ("")
    objText.writeline ("")
    strTemp = "[Abschnitt " + Trim(Str(iSeg)) + "]"   ' Abschnitt Nr... als Titel
    objText.writeline (strTemp)

    While ((bNext = True) And (FnLongOf2Bytes(yHi, yLo) < (OFFSET_ABSCHN + iSeg * LAENGE_ABSCHN)))
      
      lProp = FnLongOf2Bytes(yHi, yLo)                ' Property als Long, bequemer
                                                      ' gültige Property mit vorhandenem Text in diesem Abschnitt ?
      If ((lProp >= (OFFSET_ABSCHN + (iSeg - 1) * LAENGE_ABSCHN)) And (GetTextOfProperty(yHi, yLo) <> "")) Then
        
        strTemp = GetTextOfProperty(yHi, yLo)         ' Text zur Property mit zugehörigem Wert auf eine Zeile schreiben
                                                      ' keine der LED-Properties, normale Darstellung
        If (((lProp - (iSeg - 1) * LAENGE_ABSCHN) <> I_STATUS_LEDS_) And ((lProp - (iSeg - 1) * LAENGE_ABSCHN) <> U_STATUS_LEDS_)) Then
        
          ' neu
          strAux = Trim(CStr(GetDataFromCapsule(yHi, yLo) / GetScaleOfPar(yHi, yLo)))
          strAux = ConvertNumberToLocaleString(strAux, lThisID)   ' Zahlen immer mit Dezimalpunkt ablegen
          strTemp = strTemp & "=" & strAux
          ' ende neu
        
          ' alte version
          ' strTemp = strTemp & "=" & Trim(CStr(GetDataFromCapsule(yHi, yLo) / GetScaleOfPar(yHi, yLo)))
          ' ende alt
        Else                                          ' LED - Property, immer dreistellig anzeigen
          strAux = Trim(CStr(GetDataFromCapsule(yHi, yLo) / GetScaleOfPar(yHi, yLo)))
          strAux = Format(strAux, "000")
          strTemp = strTemp & "=" & strAux
        End If
        
        objText.writeline (strTemp)                   ' zusammengesetzte Zeile schreiben
      
      End If
      
      yHiPrev = yHi
      yLoPrev = yLo
      bNext = GetNextPar(yHiPrev, yLoPrev, yHi, yLo)  ' nächste Property abfragen
    
    Wend
  
  Next iSeg                                           ' nächsten Abschnitt bearbeiten
  
  
  '----------------------- den Kommentar-Text schreiben, falls vorhanden -------------------------------------
  
  If (g_strCommentText <> "") Then
    objText.writeline ("")
    objText.writeline ("")
    objText.writeline ("[Start Kommentar]")           ' Kommentar-Dings
    objText.write (g_strCommentText)                  ' ganzen Kommentar "ungeschminkt" reinschreiben
    objText.writeline ("")
    objText.writeline ("[Ende Kommentar]")            ' Ende Kommentar-Dings
  End If
  
  
  objText.Close                       ' Datei schliessen
     
  Exit Function                       ' alles ok, nix Error Handling


LBL_PUT_PROFILE_ERR_HANDLER:

  Call DisplayErrOrMessage(Err.Number, "")
  On Error GoTo 0                     ' Fehlerbehandlung deaktivieren (???)
  PutProfileToFile = False            ' Fehler in Funktion, Abbruch

End Function                      ' Ende PutProfileToFile





''''''''''''''''''''''''''''''''''' Ende (aktiver) Kennlinien - Source - Code ''''''''''''''''''''''''''''''''''''






''''''''''''''''''''''''''''''''''' Beginn Kennlinien - Code "Artefakte" ''''''''''''''''''''''''''''''''''''





''''''''''''''''''''''''''''''''''''''''''''''
' Fenster abschiessen
Private Sub Form_Unload(Cancel As Integer)
  ' braucht 's das? Stört wahrscheinlich nicht ;-)
  Unload Me                                   ' Formular entladen
  End                                         ' alles beenden
  
End Sub



' Monitor Fenster anzeigen
Private Sub monitor_maske_Click()
  monitor1.Show
End Sub


