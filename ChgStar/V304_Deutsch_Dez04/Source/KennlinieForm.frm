VERSION 5.00
Object = "{D940E4E4-6079-11CE-88CB-0020AF6845F6}#1.5#0"; "cwui.ocx"
Object = "{648A5603-2C6E-101B-82B6-000000000014}#1.1#0"; "MSCOMM32.OCX"
Object = "{F9043C88-F6F2-101A-A3C9-08002B2F49FB}#1.2#0"; "COMDLG32.OCX"
Begin VB.Form Kennlinie 
   BackColor       =   &H00E0E0E0&
   Caption         =   "ChargeStar NLG 5"
   ClientHeight    =   9960
   ClientLeft      =   165
   ClientTop       =   510
   ClientWidth     =   13485
   FillColor       =   &H00FF0000&
   ForeColor       =   &H80000011&
   Icon            =   "KennlinieForm.frx":0000
   LinkTopic       =   "Form3"
   ScaleHeight     =   9960
   ScaleMode       =   0  'Benutzerdefiniert
   ScaleWidth      =   16149.71
   StartUpPosition =   1  'Fenstermitte
   Begin VB.CommandButton AbschnVor 
      Caption         =   "Weiter >>"
      Height          =   375
      Left            =   8280
      TabIndex        =   105
      Top             =   4140
      Width           =   1215
   End
   Begin VB.CommandButton AbschnZurueck 
      Caption         =   " << Zurück   "
      Height          =   375
      Left            =   3720
      TabIndex        =   104
      Top             =   4140
      Width           =   1215
   End
   Begin VB.Frame Frame2 
      Caption         =   "Frame2"
      Height          =   255
      Left            =   600
      TabIndex        =   103
      Top             =   480
      Width           =   15
   End
   Begin VB.Timer time_out_z 
      Enabled         =   0   'False
      Interval        =   3000
      Left            =   12840
      Top             =   6720
   End
   Begin VB.ComboBox nlg_version 
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   12
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   420
      ItemData        =   "KennlinieForm.frx":0ECA
      Left            =   7920
      List            =   "KennlinieForm.frx":0EE0
      TabIndex        =   99
      Top             =   1200
      Width           =   975
   End
   Begin MSComDlg.CommonDialog datei_fenster1 
      Left            =   12360
      Top             =   6720
      _ExtentX        =   847
      _ExtentY        =   847
      _Version        =   393216
      CancelError     =   -1  'True
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0C000&
      Height          =   285
      Index           =   28
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   85
      TabStop         =   0   'False
      Text            =   "Nennspannung der Batterie   [Volt]"
      Top             =   600
      Width           =   4000
   End
   Begin VB.TextBox Text3 
      BackColor       =   &H00C0C000&
      Height          =   285
      Index           =   28
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   84
      TabStop         =   0   'False
      Text            =   "Temperatur-Koeffizient der Gasungsspannung   [%/°C]"
      Top             =   960
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0C000&
      Height          =   285
      Index           =   29
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   83
      TabStop         =   0   'False
      Text            =   "Bezugstemperatur für Spannungsangaben   [°C]"
      Top             =   1440
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0C000&
      Height          =   285
      Index           =   30
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   82
      TabStop         =   0   'False
      Text            =   "maximale Netzleistung   [Watt]"
      Top             =   1920
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0C000&
      Height          =   285
      Index           =   31
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   81
      TabStop         =   0   'False
      Text            =   "maximaler Netzstrom   [Amp.]"
      Top             =   2280
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0C000&
      Height          =   285
      Index           =   32
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   80
      TabStop         =   0   'False
      Text            =   "Batterie-Temperatur-Derating   [Amp./°C]"
      Top             =   3120
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0C000&
      Height          =   285
      Index           =   33
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   79
      TabStop         =   0   'False
      Text            =   "Bezugstemperatur für Derating auf Null [°C]"
      Top             =   3480
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0FFFF&
      Height          =   285
      Index           =   35
      Left            =   6960
      Locked          =   -1  'True
      TabIndex        =   78
      TabStop         =   0   'False
      Text            =   "Batterie-Temperatur über   [°C]"
      Top             =   1920
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0FFFF&
      Height          =   285
      Index           =   36
      Left            =   6960
      Locked          =   -1  'True
      TabIndex        =   77
      TabStop         =   0   'False
      Text            =   "Batterie-Spannung über   [Volt]"
      Top             =   2280
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0FFFF&
      Height          =   285
      Index           =   37
      Left            =   6960
      Locked          =   -1  'True
      TabIndex        =   76
      TabStop         =   0   'False
      Text            =   "Bezogene Netz-Energie mehr als   [kWh]"
      Top             =   2640
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0FFFF&
      Height          =   285
      Index           =   38
      Left            =   6960
      Locked          =   -1  'True
      TabIndex        =   75
      TabStop         =   0   'False
      Text            =   "Batterie-Ladung mehr als   [Ah]"
      Top             =   3000
      Width           =   4000
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H00C0FFFF&
      Height          =   285
      Index           =   39
      Left            =   6960
      Locked          =   -1  'True
      TabIndex        =   74
      TabStop         =   0   'False
      Text            =   "Ladezeit länger als   [Stunden]"
      Top             =   3360
      Width           =   4000
   End
   Begin VB.ComboBox Reduktions_art 
      BackColor       =   &H00FFFFFF&
      Height          =   315
      ItemData        =   "KennlinieForm.frx":0F02
      Left            =   4680
      List            =   "KennlinieForm.frx":0F0C
      TabIndex        =   73
      Top             =   2760
      Width           =   1695
   End
   Begin VB.TextBox textt 
      BackColor       =   &H00C0C000&
      Height          =   285
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   72
      TabStop         =   0   'False
      Text            =   "Temperatur-Reduktions-Art"
      Top             =   2760
      Width           =   4000
   End
   Begin VB.CommandButton CmdDelAllProfiles 
      Caption         =   "Die ganze Kennlinie zurücksetzen"
      Height          =   735
      Left            =   12000
      TabIndex        =   71
      ToolTipText     =   "Auf Standard Werten zurück setzen"
      Top             =   4800
      Width           =   1095
   End
   Begin VB.CommandButton CmdDelAbschnitt 
      Caption         =   "Diesen Abschnitt zurücksetzen"
      Height          =   735
      Left            =   12000
      TabIndex        =   70
      ToolTipText     =   "Auf Standard Werten zurück setzen"
      Top             =   5640
      Width           =   1095
   End
   Begin VB.CommandButton CmdDelFwOpt 
      Caption         =   "Bedingungen löschen"
      Height          =   495
      Left            =   11280
      TabIndex        =   66
      Top             =   7275
      Width           =   1815
   End
   Begin VB.CommandButton del_all_rw 
      Caption         =   "Bedingungen löschen"
      Height          =   495
      Left            =   4560
      TabIndex        =   65
      Top             =   7275
      Width           =   1815
   End
   Begin VB.ComboBox ComboWechselOption 
      BackColor       =   &H00FFFFFF&
      Height          =   315
      ItemData        =   "KennlinieForm.frx":0F25
      Left            =   6240
      List            =   "KennlinieForm.frx":0F41
      TabIndex        =   54
      Text            =   "Combo2"
      Top             =   6570
      Width           =   1575
   End
   Begin VB.ListBox BedingTypVW 
      Appearance      =   0  '2D
      BackColor       =   &H00FFFFFF&
      Height          =   810
      Left            =   6960
      TabIndex        =   53
      Top             =   6960
      Width           =   4215
   End
   Begin VB.ListBox BedingTypRW 
      Appearance      =   0  '2D
      BackColor       =   &H00FFFFFF&
      Height          =   810
      Left            =   360
      TabIndex        =   52
      ToolTipText     =   "Auswahl mit Doppelklick"
      Top             =   6960
      Width           =   4095
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   5
      Left            =   4440
      TabIndex        =   42
      Top             =   9360
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWButton CWButton2 
      Height          =   375
      Index           =   4
      Left            =   6000
      TabIndex        =   24
      Top             =   8760
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      Caption         =   "Q1 Und Q2"
      Height          =   255
      Index           =   4
      Left            =   9960
      TabIndex        =   23
      Top             =   5880
      Width           =   1575
   End
   Begin VB.CheckBox Check1 
      Caption         =   "Quittiersignal2"
      Height          =   255
      Index           =   3
      Left            =   9960
      TabIndex        =   22
      Top             =   5640
      Width           =   1575
   End
   Begin VB.CheckBox Check1 
      Caption         =   "Quittiersignal1"
      Height          =   255
      Index           =   2
      Left            =   9960
      TabIndex        =   21
      Top             =   5400
      Width           =   1575
   End
   Begin VB.CheckBox Check1 
      Caption         =   "Aus/Einschalten"
      Height          =   255
      Index           =   1
      Left            =   9960
      TabIndex        =   20
      Top             =   5160
      Width           =   1575
   End
   Begin CWUIControlsLib.CWNumEdit num1 
      Height          =   285
      Index           =   4
      Left            =   7440
      TabIndex        =   18
      Top             =   5520
      Width           =   975
      _Version        =   196640
      _ExtentX        =   1720
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   ".%"
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin VB.TextBox Text8 
      BackColor       =   &H80000013&
      Height          =   285
      Left            =   5280
      Locked          =   -1  'True
      TabIndex        =   16
      Text            =   "Zählt für Überladung"
      Top             =   5520
      Width           =   2055
   End
   Begin VB.TextBox Text7 
      Alignment       =   2  'Zentriert
      BackColor       =   &H80000013&
      Height          =   285
      Left            =   8640
      Locked          =   -1  'True
      TabIndex        =   15
      Text            =   "Status"
      Top             =   5160
      Width           =   975
   End
   Begin VB.TextBox Text6 
      Alignment       =   2  'Zentriert
      BackColor       =   &H80000013&
      Height          =   285
      Left            =   3960
      Locked          =   -1  'True
      TabIndex        =   13
      Text            =   "Status"
      Top             =   5160
      Width           =   975
   End
   Begin VB.TextBox Text3 
      BackColor       =   &H80000013&
      Height          =   285
      Index           =   0
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   12
      Text            =   "Zählt für Überladung"
      Top             =   5520
      Width           =   2055
   End
   Begin VB.ComboBox ComboAbschnitt 
      BackColor       =   &H00FFFFFF&
      Height          =   315
      Left            =   5280
      TabIndex        =   11
      Text            =   "Abschnitt"
      Top             =   4170
      Width           =   1335
   End
   Begin CWUIControlsLib.CWButton CWButton1 
      Height          =   255
      Index           =   1
      Left            =   3960
      TabIndex        =   4
      Top             =   5520
      Width           =   255
      _Version        =   196640
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      C[0]_18         =   255
      C[1]_18         =   255
      Image_18        =   19
      ClassName_19    =   "CCWPictImage"
      opts_19         =   1280
      Rows_19         =   1
      Cols_19         =   1
      Pict_19         =   74
      F_19            =   255
      B_19            =   255
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
      Height          =   285
      Index           =   2
      Left            =   7440
      TabIndex        =   3
      Top             =   5160
      Width           =   975
      _Version        =   196640
      _ExtentX        =   1720
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0 ""V"""
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
      RangeMax_Val_1  =   999
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit num1 
      Height          =   285
      Index           =   1
      Left            =   2760
      TabIndex        =   2
      Top             =   5160
      Width           =   975
      _Version        =   196640
      _ExtentX        =   1720
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   "0.0 ""A"""
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
      RangeMax_Val_1  =   999
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H80000013&
      Height          =   285
      Index           =   2
      Left            =   5280
      Locked          =   -1  'True
      TabIndex        =   1
      TabStop         =   0   'False
      Text            =   "Maximale Ladespannung"
      Top             =   5160
      Width           =   2055
   End
   Begin VB.TextBox Text1 
      BackColor       =   &H80000013&
      Height          =   285
      Index           =   1
      Left            =   600
      Locked          =   -1  'True
      TabIndex        =   0
      TabStop         =   0   'False
      Text            =   "Maximaler Ladestrom"
      Top             =   5160
      Width           =   2055
   End
   Begin CWUIControlsLib.CWButton CWButton1 
      Height          =   255
      Index           =   2
      Left            =   4320
      TabIndex        =   5
      Top             =   5520
      Width           =   255
      _Version        =   196640
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      C[0]_19         =   255
      C[1]_19         =   255
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   255
      B_20            =   255
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
   Begin CWUIControlsLib.CWButton CWButton1 
      Height          =   255
      Index           =   3
      Left            =   4680
      TabIndex        =   6
      Top             =   5520
      Width           =   255
      _Version        =   196640
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      C[0]_19         =   255
      C[1]_19         =   255
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   255
      B_20            =   255
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
   Begin CWUIControlsLib.CWButton CWButton1 
      Height          =   255
      Index           =   4
      Left            =   8640
      TabIndex        =   7
      Top             =   5520
      Width           =   255
      _Version        =   196640
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      C[0]_19         =   255
      C[1]_19         =   255
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   255
      B_20            =   255
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
   Begin CWUIControlsLib.CWButton CWButton1 
      Height          =   255
      Index           =   5
      Left            =   9000
      TabIndex        =   8
      Top             =   5520
      Width           =   255
      _Version        =   196640
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      C[0]_19         =   255
      C[1]_19         =   255
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   255
      B_20            =   255
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
   Begin CWUIControlsLib.CWButton CWButton1 
      Height          =   255
      Index           =   6
      Left            =   9360
      TabIndex        =   9
      Top             =   5520
      Width           =   255
      _Version        =   196640
      _ExtentX        =   450
      _ExtentY        =   450
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      Boolean_0       =   1
      ClassName_1     =   "CCWBoolean"
      opts_1          =   2606
      C[0]_1          =   -2147483643
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
      C[0]_19         =   255
      C[1]_19         =   255
      Image_19        =   20
      ClassName_20    =   "CCWPictImage"
      opts_20         =   1280
      Rows_20         =   1
      Cols_20         =   1
      Pict_20         =   74
      F_20            =   255
      B_20            =   255
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
      Height          =   1335
      Left            =   9840
      TabIndex        =   10
      Top             =   4920
      Width           =   1935
   End
   Begin VB.Frame Frame1 
      Caption         =   "U-PHASE"
      Height          =   1335
      Index           =   1
      Left            =   5160
      TabIndex        =   17
      Top             =   4920
      Width           =   4575
   End
   Begin CWUIControlsLib.CWNumEdit num1 
      Height          =   285
      Index           =   3
      Left            =   2760
      TabIndex        =   19
      Top             =   5520
      Width           =   975
      _Version        =   196640
      _ExtentX        =   1720
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      format_1        =   2
      ClassName_2     =   "CCWFormat"
      Format_2        =   ".%"
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin VB.Frame Frame1 
      Caption         =   "I-PHASE"
      Height          =   1335
      Index           =   0
      Left            =   360
      TabIndex        =   14
      Top             =   4920
      Width           =   4695
   End
   Begin CWUIControlsLib.CWButton CWButton2 
      Height          =   375
      Index           =   5
      Left            =   6000
      TabIndex        =   25
      Top             =   9120
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      Left            =   6000
      TabIndex        =   26
      Top             =   8040
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
      Left            =   6000
      TabIndex        =   27
      Top             =   8400
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
   Begin CWUIControlsLib.CWButton CWButton3 
      Height          =   375
      Index           =   5
      Left            =   12720
      TabIndex        =   28
      Top             =   9120
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
   Begin CWUIControlsLib.CWButton CWButton3 
      Height          =   375
      Index           =   2
      Left            =   12720
      TabIndex        =   29
      Top             =   8040
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
   Begin CWUIControlsLib.CWButton CWButton3 
      Height          =   375
      Index           =   3
      Left            =   12720
      TabIndex        =   30
      Top             =   8400
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
   Begin CWUIControlsLib.CWButton CWButton3 
      Height          =   375
      Index           =   4
      Left            =   12720
      TabIndex        =   31
      Top             =   8760
      Visible         =   0   'False
      Width           =   375
      _Version        =   196640
      _ExtentX        =   661
      _ExtentY        =   661
      _StockProps     =   68
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.24
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
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
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   1
      Left            =   4440
      TabIndex        =   43
      Top             =   7920
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   2
      Left            =   4440
      TabIndex        =   44
      Top             =   8280
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   3
      Left            =   4440
      TabIndex        =   45
      Top             =   8640
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit rueckw 
      Height          =   255
      Index           =   4
      Left            =   4440
      TabIndex        =   46
      Top             =   9000
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit weiterw 
      Height          =   255
      Index           =   5
      Left            =   11160
      TabIndex        =   60
      Top             =   9360
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit weiterw 
      Height          =   255
      Index           =   1
      Left            =   11160
      TabIndex        =   61
      Top             =   7920
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit weiterw 
      Height          =   255
      Index           =   2
      Left            =   11160
      TabIndex        =   62
      Top             =   8280
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit weiterw 
      Height          =   255
      Index           =   3
      Left            =   11160
      TabIndex        =   63
      Top             =   8640
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit weiterw 
      Height          =   255
      Index           =   4
      Left            =   11160
      TabIndex        =   64
      Top             =   9000
      Visible         =   0   'False
      Width           =   855
      _Version        =   196640
      _ExtentX        =   1508
      _ExtentY        =   450
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   393278
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin MSCommLib.MSComm MSComm1 
      Left            =   11760
      Top             =   6600
      _ExtentX        =   1005
      _ExtentY        =   1005
      _Version        =   393216
      DTREnable       =   -1  'True
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   0
      Left            =   4680
      TabIndex        =   86
      Top             =   600
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   1
      Left            =   4680
      TabIndex        =   87
      Top             =   960
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      RangeMin_Val_1  =   -0.3
      RangeMaxVarType_1=   5
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   2
      Left            =   4680
      TabIndex        =   88
      Top             =   1440
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      RangeMax_Val_1  =   127
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   3
      Left            =   4680
      TabIndex        =   89
      Top             =   1920
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   4
      Left            =   4680
      TabIndex        =   90
      Top             =   2280
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      RangeMax_Val_1  =   99
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   5
      Left            =   4680
      TabIndex        =   91
      Top             =   3120
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   6
      Left            =   4680
      TabIndex        =   92
      Top             =   3480
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      RangeMaxVarType_1=   5
      RangeMax_Val_1  =   127
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   7
      Left            =   11160
      TabIndex        =   93
      Top             =   1920
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      RangeMax_Val_1  =   127
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   8
      Left            =   11160
      TabIndex        =   94
      Top             =   2280
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      RangeMax_Val_1  =   2999
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   9
      Left            =   11160
      TabIndex        =   95
      Top             =   2640
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   10
      Left            =   11160
      TabIndex        =   96
      Top             =   3000
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin CWUIControlsLib.CWNumEdit CWNumEdit1 
      Height          =   285
      Index           =   11
      Left            =   11160
      TabIndex        =   97
      Top             =   3360
      Width           =   1665
      _Version        =   196640
      _ExtentX        =   2937
      _ExtentY        =   503
      _StockProps     =   4
      BeginProperty Font {0BE35203-8F91-11CE-9DE3-00AA004BB851} 
         Name            =   "MS Sans Serif"
         Size            =   8.25
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Reset_0         =   0   'False
      CompatibleVers_0=   196640
      NumEdit_0       =   1
      ClassName_1     =   "CCWNumEdit"
      opts_1          =   458814
      C[0]_1          =   16777215
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
      ButtonStyle_1   =   0
      Bindings_1      =   4
      ClassName_4     =   "CCWBindingHolderArray"
      Editor_4        =   5
      ClassName_5     =   "CCWBindingHolderArrayEditor"
      Owner_5         =   1
   End
   Begin VB.Label Label8 
      BackColor       =   &H00C0FFFF&
      Caption         =   " Abschaltung in jedem Fall, wenn:"
      Height          =   255
      Left            =   9480
      TabIndex        =   108
      Top             =   1560
      Width           =   2535
   End
   Begin VB.Shape Shape6 
      BackColor       =   &H00000000&
      BorderColor     =   &H0000FFFF&
      Height          =   2055
      Left            =   6840
      Top             =   1680
      Width           =   6135
   End
   Begin VB.Label Label7 
      BackColor       =   &H00E0E0E0&
      Height          =   255
      Left            =   5175
      TabIndex        =   107
      Top             =   4200
      Width           =   135
   End
   Begin VB.Shape Shape5 
      BorderColor     =   &H00C00000&
      Height          =   455
      Left            =   5160
      Top             =   4100
      Width           =   2895
   End
   Begin VB.Label Label6 
      Caption         =   " Allgemeine Einstellungen"
      Height          =   255
      Left            =   600
      TabIndex        =   106
      Top             =   120
      Width           =   1935
   End
   Begin VB.Shape Shape3 
      Height          =   3615
      Left            =   120
      Top             =   240
      Width           =   13215
   End
   Begin VB.Label KennlAbschnLabel 
      BackColor       =   &H00E0E0E0&
      Caption         =   "  der Ladekennlinie"
      Height          =   255
      Left            =   6600
      TabIndex        =   102
      Top             =   4215
      Width           =   1455
   End
   Begin VB.Label Label5 
      Caption         =   " Wechsel - Bedingung:"
      Height          =   255
      Left            =   4560
      TabIndex        =   101
      Top             =   6600
      Width           =   1695
   End
   Begin VB.Label status_fenster 
      Caption         =   "Not Connected"
      Height          =   735
      Left            =   6840
      TabIndex        =   100
      Top             =   360
      Width           =   6135
   End
   Begin VB.Label Label4 
      BackColor       =   &H00808000&
      Caption         =   "NLG"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   13.5
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   375
      Left            =   6840
      TabIndex        =   98
      Top             =   1200
      Width           =   975
   End
   Begin VB.Shape AbschnitteRahmen 
      BorderColor     =   &H00C00000&
      Height          =   5535
      Left            =   120
      Top             =   4320
      Width           =   13215
   End
   Begin VB.Label Label3 
      Caption         =   " Generell zu diesem Abschnitt"
      Height          =   255
      Left            =   480
      TabIndex        =   69
      Top             =   4560
      Width           =   2175
   End
   Begin VB.Label Label2 
      Alignment       =   2  'Zentriert
      BackColor       =   &H0000C000&
      Caption         =   "Zum nächsten Abschnitt, wenn: >>>>>"
      Height          =   255
      Left            =   8280
      TabIndex        =   68
      Top             =   6600
      Width           =   3135
   End
   Begin VB.Label Label1 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      Caption         =   "<<<<< Zum vorherigen Abschnitt, wenn:"
      Height          =   255
      Left            =   960
      TabIndex        =   67
      Top             =   6600
      Width           =   3135
   End
   Begin VB.Shape Shape4 
      BackColor       =   &H00FFFFFF&
      BorderColor     =   &H00FF0000&
      Height          =   1695
      Left            =   240
      Top             =   4680
      Width           =   11655
   End
   Begin VB.Shape Shape2 
      BorderColor     =   &H0000C000&
      Height          =   3015
      Left            =   6960
      Top             =   6720
      Width           =   6255
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "4"
      Height          =   255
      Index           =   4
      Left            =   360
      TabIndex        =   36
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   9000
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "3"
      Height          =   255
      Index           =   3
      Left            =   360
      TabIndex        =   35
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   8640
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "2"
      Height          =   255
      Index           =   2
      Left            =   360
      TabIndex        =   34
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   8280
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "1"
      Height          =   255
      Index           =   1
      Left            =   360
      TabIndex        =   33
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   7920
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label vorher 
      BackColor       =   &H008080FF&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "5"
      Height          =   255
      Index           =   5
      Left            =   360
      TabIndex        =   32
      ToolTipText     =   "Bewegen mit Linke Maustaste gedruckt"
      Top             =   9360
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label weitereinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H0000C000&
      Height          =   255
      Index           =   4
      Left            =   12000
      TabIndex        =   59
      Top             =   9000
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label weitereinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H0000C000&
      Height          =   255
      Index           =   3
      Left            =   12000
      TabIndex        =   58
      Top             =   8640
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label weitereinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H0000C000&
      Height          =   255
      Index           =   2
      Left            =   12000
      TabIndex        =   57
      Top             =   8280
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label weitereinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H0000C000&
      Height          =   255
      Index           =   1
      Left            =   12000
      TabIndex        =   56
      Top             =   7920
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label weitereinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H0000C000&
      Height          =   255
      Index           =   5
      Left            =   12000
      TabIndex        =   55
      Top             =   9360
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      Height          =   255
      Index           =   4
      Left            =   5280
      TabIndex        =   51
      Top             =   9000
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      Height          =   255
      Index           =   3
      Left            =   5280
      TabIndex        =   50
      Top             =   8640
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      Height          =   255
      Index           =   2
      Left            =   5280
      TabIndex        =   49
      Top             =   8280
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      Height          =   255
      Index           =   1
      Left            =   5280
      TabIndex        =   48
      Top             =   7920
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label rueckeinheit 
      Alignment       =   2  'Zentriert
      BackColor       =   &H008080FF&
      Height          =   255
      Index           =   5
      Left            =   5280
      TabIndex        =   47
      Top             =   9360
      Visible         =   0   'False
      Width           =   735
   End
   Begin VB.Label nexte 
      BackColor       =   &H0000C000&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "4"
      Height          =   255
      Index           =   4
      Left            =   7080
      TabIndex        =   41
      Top             =   9000
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label nexte 
      BackColor       =   &H0000C000&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "3"
      Height          =   255
      Index           =   3
      Left            =   7080
      TabIndex        =   40
      Top             =   8640
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label nexte 
      BackColor       =   &H0000C000&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "2"
      Height          =   255
      Index           =   2
      Left            =   7080
      TabIndex        =   39
      Top             =   8280
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label nexte 
      BackColor       =   &H0000C000&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "1"
      Height          =   255
      Index           =   1
      Left            =   7080
      TabIndex        =   38
      Top             =   7920
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Label nexte 
      BackColor       =   &H0000C000&
      BorderStyle     =   1  'Fest Einfach
      Caption         =   "5"
      Height          =   255
      Index           =   5
      Left            =   7080
      TabIndex        =   37
      Top             =   9360
      Visible         =   0   'False
      Width           =   4095
   End
   Begin VB.Shape Shape1 
      BackColor       =   &H00FFFFFF&
      BorderColor     =   &H008080FF&
      Height          =   3015
      Left            =   240
      Top             =   6720
      Width           =   6255
   End
   Begin VB.Menu Profile 
      Caption         =   "&Kennlinie"
      Index           =   10
      Begin VB.Menu new_profile 
         Caption         =   "&Neu"
         Shortcut        =   ^N
      End
      Begin VB.Menu load_profile 
         Caption         =   "&Laden"
         Shortcut        =   ^O
      End
      Begin VB.Menu save_profile 
         Caption         =   "&Speichern"
         Shortcut        =   ^S
      End
      Begin VB.Menu ligne_blank 
         Caption         =   "-"
      End
      Begin VB.Menu Profile_from 
         Caption         =   "&vom NLG holen"
         Shortcut        =   {F6}
      End
      Begin VB.Menu Profile_to 
         Caption         =   "&zum NLG übertragen"
         Shortcut        =   {F7}
      End
      Begin VB.Menu blanc_0 
         Caption         =   "-"
      End
      Begin VB.Menu Quitter 
         Caption         =   "&Beenden"
      End
   End
   Begin VB.Menu Edit1 
      Caption         =   "&Bearbeiten"
      Begin VB.Menu dele_option 
         Caption         =   "Bedingung &löschen"
      End
      Begin VB.Menu ligne_blank2 
         Caption         =   "-"
      End
      Begin VB.Menu abschnitt_zurueck 
         Caption         =   "Abschnitt &zurücksetzen"
      End
   End
   Begin VB.Menu extra1 
      Caption         =   "E&xtras"
      Begin VB.Menu connect_nlg 
         Caption         =   "mit NLG &verbinden"
         Shortcut        =   {F4}
      End
      Begin VB.Menu disconnect_nlg 
         Caption         =   "Verbindung &trennen"
         Shortcut        =   {F5}
      End
      Begin VB.Menu info_nlg 
         Caption         =   "&Geräte  Info"
      End
      Begin VB.Menu blank3 
         Caption         =   "-"
      End
      Begin VB.Menu monitor_maske 
         Caption         =   "&Monitor"
         Shortcut        =   ^M
      End
      Begin VB.Menu blank5 
         Caption         =   "-"
      End
      Begin VB.Menu connect_optionen 
         Caption         =   "Verbindungs-&Optionen"
      End
   End
   Begin VB.Menu Help 
      Caption         =   "&Hilfe"
      Begin VB.Menu HelpContents 
         Caption         =   "&Inhalt"
         Shortcut        =   {F1}
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
' erstellt  / Okt. '01  / Quang H.    / Rohfassung, funktioniert im Wesentlichen                                '
' geändert  / Nov. '01  / Beat G.     / Kommentare eingefügt, Bezeichnungen und Look&Feel angepasst             '
' bg001     / 16.11.01  / Beat G.     / Datenhaltung und Speicherung der Kennlinien korrigiert                  '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
'                                     /                                                                         '
' InitPosPropArrays                   / Arrays für Verknüpfung GUI - Datenkapsel initialisieren                 '
' PutGuiValToCapsule                  / Im GUI angezeigten Wert in die Kapsel schreiben                         '
' GetGuiValFromCapsule                / Wert für GUI - Anzeige aus der Kapsel holen                             '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'
' Anmerkung zur Datenhaltung der vorwärts/rückwärts - Bedingungen:
' "nexte/vorher" ist der Text, "weiterw/rueckw" der Wert, "weitereinheit/rueckeinheit" die Einheit der..
'..Bedingung und "CWButton2/3" die logische Verknüpfung zwischen 2 Bedingungen; als Datenbehälter dienen..
'..die o.g. Controls im Formular, wobei diese dann später wohl nach rueck / weiter kopiert werden.
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
' hier stehen die Index-Nummern der GUI-Controls, die zum entsprechenden Par gehören
' die "allgemeinen Einstellungen", soweit sie Index-adressiert sind (Ausnahme: Temp-Reduktions-Art und NLG-Typ)
Const GUI_BATT_NENN As Integer = 0    ' Batterie - Nennspannung
Const GUI_TEMP_KOEF As Integer = 1    ' Temperatur - Koeff. Gasung
Const GUI_BEZUG_TMP As Integer = 2    ' Bezugs - Temp Spannungen
Const GUI_MAX_P_NEZ As Integer = 3    ' Max. Netz - Leistung
Const GUI_MAX_I_NEZ As Integer = 4    ' Max. Netz - Strom
Const GUI_DERAT_HYS As Integer = 5    ' Temp. Derating
Const GUI_BEZG_HYST As Integer = 6    ' Temp. Derating Bezugstemp
' die Sofort - Abschalt - Kriterien
Const GUI_TMP_UEBER As Integer = 7    ' Über Temp. Batterie
Const GUI_SPG_UEBER As Integer = 8    ' Über Spannung Batterie
Const GUI_ENERG_UEB As Integer = 9    ' Netz-Energie mehr als .. kWh
Const GUI_LADEN_UEB As Integer = 10   ' Batt. Ladung mehr als .. Ah
Const GUI_LADE_ZEIT As Integer = 11   ' Ladezeit länger als .. h
' die U-Phase / I-Phase Parameter
Const GUI_I_PH_MAX  As Integer = 1    ' I-Phase max. Ladestrom
Const GUI_U_PH_MAX  As Integer = 2    ' U-Phase max. Ladespg
Const GUI_I_PH_UEB  As Integer = 3    ' I-Phase Überladung %
Const GUI_U_PH_UEB  As Integer = 4    ' U-Phase Überladung %
Const GUI_I_PH_STL  As Integer = 1    ' I-Phase Status LED links
Const GUI_I_PH_STM  As Integer = 2    ' I-Phase Status LED mitte
Const GUI_I_PH_STR  As Integer = 3    ' I-Phase Status LED rechts
Const GUI_U_PH_STL  As Integer = 4    ' U-Phase Status LED links
Const GUI_U_PH_STM  As Integer = 5    ' U-Phase Status LED mitte
Const GUI_U_PH_STR  As Integer = 6    ' U-Phase Status LED rechts
Const GUI_U_PH_AEG  As Integer = 1    ' zurück zu Abschnitt 1 wenn Aus/Ein
Const GUI_U_PH_SG1  As Integer = 2    ' zurück zu Abschnitt 1 bei Signal 1
Const GUI_U_PH_SG2  As Integer = 3    ' zurück zu Abschnitt 1 bei Signal 2
Const GUI_U_PH_S12  As Integer = 4    ' zurück zu Abschnitt 1 bei Signal 1 und 2

'abc
'
' generell gilt: Wechsel Option = Bedingung für Abschnittswechsel = Kriterium
'
'-------------------------------------- globales Datenfeld ----------------------------------------'
'
' 6 leds mit 3 status
Dim leds(ANZ_LADEABSCHNITTE, 6) As Integer             ' Datenfeld 8x7, 7 Abschnitte x 6 LEDS?
Dim sourceidxv As Integer, sourceidxw As Integer ' wahrscheinlich für Bedingungs - Kopieren gebraucht
Dim abschnitt_generell(1 To ANZ_LADEABSCHNITTE) As abschnittg  ' Generelle Daten (Umax, Imax etc.) des Abschnitts
Dim rueck(ANZ_LADEABSCHNITTE, 5) As kriterium          ' Rückwärts, 7 Abschnitte à 5 Bedingungen (1-7, 0-71)
Dim weiter(ANZ_LADEABSCHNITTE, 5) As kriterium         ' Vorwärts, 7 Abschnitte à 5 Bedingungen (1-7, 0-71)
Dim rechte_source As Boolean          ' ?
' bg001 von Integer zu Variant geändert, weil's bei manchen Werten auch Dezimalstellen gibt
' Dim nlgx(12) As Variant               ' 12 "Allgemeine Einstellungen"

Dim datei_laden As String             ' zu ladende Datei
Dim datei_speichern As String         ' zu speichernde Datei

Dim karakter As String                ' ein Tastaturzeichen
Dim inbuffer As String                ' Input String
Dim end_zeichen As Boolean            ' String Terminator
Dim retour As String                  ' Rückgabewert der String Erfassung
Dim time_out As Boolean               ' Timeout (auch Timeout_z)
'
'
' ------------------ die Arrays (oder Einzel-Var), welche Controls zu Parameter-ID's zuordnen ---------
Dim lGuiParArrayAllg(11)  As Long     ' die indizierten Par der allgemeinen Einstellungen
Dim lGuiParRedArt         As Long     ' die Temperatur Reduktions-Art
Dim lGuiParNLGTyp         As Long     ' der NLG Typ
' N.B. die Abschnitts-ComboBox gehört nicht dazu, sie enthält nur die Information, welcher Abschnitt gerade angezeigt wird
Dim lGuiGenAbEdBox(4)     As Long     ' Edit Boxen des "Generell" Feldes (Strom, Spannung, Ueberladung)
Dim lGuiStatusLeds(6)     As Long     ' diese 6 Felder sind 2 Par; 1 Par hat 3 Leds à 3 Zustände (0,1,2)
Dim lGuiRetCheckBox(4)    As Long     ' 4 Checkboxen (0,1), je durch einen eigenen Par repräsentiert
' N.B. die WechselBedingung-ComboBox gehört auch nicht dazu, sie enthält eine temporäre Information
Dim lGuiTypRueck(5)       As Long     ' Typ der Rückwärts - Bedingungen
Dim lGuiWertRueck(5)      As Long     ' Werte der Rückwärts - Bedingungen
Dim lGuiAndOrRueck(4)     As Long     ' Verknüpfungen zwischen Rückwärts - Bedingungen
Dim lGuiTypVor(5)         As Long     ' Typ der Vorwärts - Bedingungen
Dim lGuiWertVor(5)        As Long     ' Werte der Vorwärts - Bedingungen
Dim lGuiAndOrVor(4)       As Long     ' Verknüpfungen zwischen Vorwärts - Bedingungen
'
Dim bInvalidCWNumEd1      As Boolean  ' besagt, dass die Werte des Controls CWNumEdit1 momentan ungültig sind




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Bedingung für "zurück zum Abschnitt 1" (aus/ein, Quittiersignale) geändert, also Parameter des aktuellen..    '
'..Abschnitts entsprechend ändern                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

' die 4 Ankreuz - Kästchen
Private Sub Check1_Click(Index As Integer)
  
  ' N.B.: um Komplikationen mit VB Extra-Würsten zu vermeiden, definieren wir Wahr als 1 und Falsch als 0
  If (Check1(Index).Value = vbChecked) Then           ' Kästchen wurde angekreuzt
    Call PutGuiValToCapsule(lGuiRetCheckBox(Index), BTRUE)      ' bg002 neue Variante: ab in die Datenkapsel damit
    Mid(abschnitt_generell(aktuelle_abschnitt).zumab1, Index, 1) = "1"
  Else                                                ' Kästchen wurde nicht angekreuzt
    Call PutGuiValToCapsule(lGuiRetCheckBox(Index), BFALSE)      ' bg002 neue Variante: ab in die Datenkapsel damit
    Mid(abschnitt_generell(aktuelle_abschnitt).zumab1, Index, 1) = "0"
  End If

End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Abschnitt der Kennlinie wählen
Private Sub ComboAbschnitt_Click()
   aktuelle_abschnitt = (ComboAbschnitt.ListIndex) + 1
   'Debug.Print aktuelle_abschnitt
   
   Call Abschnitt_wechseln
   
End Sub


' Funktion für Abschnittswechsel
Private Sub Abschnitt_wechseln()

If aktuelle_abschnitt = 1 Then          ' Beim ersten Abschnitt kann man nicht rückwärts
      BedingTypRW.Enabled = False
      AbschnZurueck.Enabled = False
    Else
      BedingTypRW.Enabled = True
      AbschnZurueck.Enabled = True
    End If
    
    If aktuelle_abschnitt = ANZ_LADEABSCHNITTE Then          ' Beim letzten Abschnitt kann man nicht vorwärts
      BedingTypVW.Enabled = False
      AbschnVor.Enabled = False
    Else
      BedingTypVW.Enabled = True
      AbschnVor.Enabled = True
    End If
    
   Call abschnitt_anzeigen(AKTUALISIEREN_ABSCHNITT)
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Typ der Bedingung für Abschnittswechsel wählen
Private Sub ComboWechselOption_Click()
  Call ListenMitBedingungenFuellen
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' rrr debug
'Private Sub Command6_Click()
'    raus2 = True
'End Sub


' Verbindung mit NLG aufnehmen
Private Sub connect_nlg_Click()
    
    Dim ret As Variant
    Dim retour As String
    
    If MSComm1.PortOpen = False Then              ' Port noch nicht geöffnet?
        MSComm1.CommPort = com_port               ' Com 1 oder 2 ?
        MSComm1.Settings = "19200,n,8,1"          ' Verbindungs - Parameter
        MSComm1.InputLen = 1
        MSComm1.RThreshold = 1
        MSComm1.SThreshold = 1
        MSComm1.PortOpen = True                   ' Port jetzt geöffnet
        
        '''Auf NLG Prüfen
        
               
        MSComm1.Output = "[$2;0]"
        
        ''' Warten bis alles gesendet ist
        Do
            ret = DoEvents()                      ' Windows Ereignisse ausführen ("kooperatives Multitasking")
        Loop Until MSComm1.OutBufferCount = 0
        
        retour = warten_auf_string(0)             ' String einlesen, alle Zeichen lesen
        
        If retour <> "" Then                      ' String nicht leer
            If Left(retour, 3) = "NLG" Then       ' wenn die ersten 3 Zeichen des Strings "NLG" heissen, hat's geantwortet
              ' Wenn die Nummer des NLG Typs innerhalb des unterstützten Bereichs liegt, geh' weiter, sonst Fehlermeldung
              If (Val(Mid(retour, 4, 3)) <= NLG_MAX_KLASSE) And (Val(Mid(retour, 4, 3)) >= NLG_MIN_KLASSE) Then
                nlg_version.ListIndex = Val(Right(retour, 1)) 'Zahlenwert des letzte Zeichens ist Listenindex
                MsgBox ("Verbindung wurde hergestellt")
                connected = True                  ' Verbindung zustande gekommen..
                Profile_to.Enabled = True         '..also Kennlinie zum Gerät schicken möglich..
                Profile_from.Enabled = True       '..und Kennlinie vom Gerät holen möglich..
                connect_nlg.Enabled = False       '..und erneut verbinden nicht möglich..
                disconnect_nlg.Enabled = True     '..und Verbindung abbrechen möglich
                Exit Sub                          ' Aus Funktion aussteigen
              Else
                MsgBox ("NLG Typ nicht erkannt")
              End If
            End If
        End If
                MsgBox ("Verbindung konnte nicht hergestellt werden ")
                connected = False
                If MSComm1.PortOpen = True Then MSComm1.PortOpen = False
        
        
    Else                                          ' Port geöffnet
        MsgBox ("Serielle Schnittstelle hat schon eine Verbindung")
    End If
    
End Sub


' Die LED-Knöpfchen können aus (grau) sein, rot blinken oder rot leuchten
' wenn man draufklickt, "schalten" sie einen Zustand weiter
Private Sub CWButton1_Click(Index As Integer)
    
  Dim led_temp As Integer
    
    Select Case leds(aktuelle_abschnitt, Index)
        Case 0                                    ' aus, grau
            CWButton1(Index).Value = True         ' umschalten auf ein, leuchten
            led_temp = 1                          ' ein
        Case 1
            CWButton1(Index).Value = True         ' ein, leuchten
            CWButton1(Index).OnImages("Image").BlinkInterval = cwSpeedFast
            led_temp = 2                          ' umschalten auf ein, blinken
            
        Case 2                                    ' ein, blinken
            CWButton1(Index).OnImages("Image").BlinkInterval = cwSpeedOff
            CWButton1(Index).Value = False        ' umschalten auf aus
            led_temp = 0                          ' aus
        
    End Select
     leds(aktuelle_abschnitt, Index) = led_temp   ' neuen Zustand an entsprechende LED zuweisen
    
End Sub


' Oder / Und - Button für Rückwärts - Bedingungen
Private Sub CWButton2_ValueChanged(Index As Integer, ByVal Value As Boolean)
    rueck(aktuelle_abschnitt, Index).mitnexte = CWButton2(Index).Value
End Sub


' Oder / Und - Button für Vorwärts - Bedingungen
Private Sub CWButton3_ValueChanged(Index As Integer, ByVal Value As Boolean)
 weiter(aktuelle_abschnitt, Index).mitnexte = CWButton3(Index).Value
End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Einen geänderten Wert der "Allgemeine Einstellungen" in die Datenkapsel übernehmen                            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub CWNumEdit1_ValueChanged(Index As Integer, Value As Variant, PreviousValue As Variant, ByVal OutOfRange As Boolean)
  
' nlgx(Index) = Value
  If (bInvalidCWNumEd1 = False) Then        ' "Semaphor" besagt, dass CWNumEdit1-Werte gerade nicht gültig sind
    Call PutGuiValToCapsule(lGuiParArrayAllg(Index), Value)   ' bg002 neue Variante: ab in die Datenkapsel damit
  End If
End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinien - Fenster laden / initialisieren                                                                   '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_Load()
    
    Dim ab_tmp As String
    Dim i As Integer
    Dim j As Integer
    Dim k As Variant
    
' globale Variablen, die initialisiert werden müssen

    Call InitPosPropArrays              ' Zuordnungen von Controls zu Prop_ID's der Parameter herstellen

' Menü - Einträge

    connected = False                   ' keine Verbindung beim Aufstarten, also..
    connect_nlg.Enabled = True          '..Verbindung herstellen möglich und..
    disconnect_nlg.Enabled = False      '..Verbindung abbrechen nicht möglich und..
    Profile_from.Enabled = False        '..und Kennline vom Gerät laden nicht möglich und..
    Profile_to.Enabled = False           '..Kennlinie zum Gerät schicken nicht möglich
    
    
    
    ' Combox mit "Abschnitt 1", "Abschnitt 2"  usw. füllen
    For i = 1 To ANZ_LADEABSCHNITTE
      ab_tmp = " Abschnitt " + Str(i)
      ComboAbschnitt.AddItem ab_tmp, i - 1
        
      abschnitt_generell(i).zumab1 = "0000"         ' bg001 alle Checkboxen für "zurück zu Abschn.1" auf 0 setzen
        
      For j = 1 To 6                                ' Datenfelder für LED's initialisieren
        leds(i, j) = 0
      Next j
    
    Next i
    
    ComboAbschnitt.ListIndex = 0                    ' beginne mit Abschnitt 1
   
    
    For i = 0 To 8
        BedingTypRW.AddItem rw(i, 0), i                   ' Liste für Rückwärts - Bedingungen
    Next i
    
    BedingTypRW.ListIndex = 0
    ComboWechselOption.ListIndex = 0
    
    nlg_version.ListIndex = 0
    
    CmdDelAbschnitt.Enabled = False
    CmdDelAllProfiles.Enabled = False
    abschnitt_zurueck.Enabled = False
    
    BedingTypRW.Enabled = False
    
    Reduktions_art.ListIndex = 0
   
   com_port = 1
    k = ""
    end_zeichen = False
    time_out = False
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Fenster abschiessen
Private Sub Form_Unload(Cancel As Integer)
    'Form1.Visible = True
    Unload Me
    End
    
End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' das About / Info - Fenster                                                                                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub HelpAbout_Click()
  frmAbout.Caption = "Über ChargeStar"
  frmAbout.Show vbModal
End Sub


' Hier wird eine kurze Einführung in die Arbeit mit ChargeStar angezeigt
Private Sub HelpContents_Click()
  MsgBox ("Hilfe-Text: noch nicht implementiert")
End Sub


' Geräte - Info holen, ist im Gerät als Vorspann zur Kennlinie abgelegt
Private Sub info_nlg_Click()
  MsgBox ("Geräte-Info: noch nicht implementiert")
End Sub


' Monitor Fenster anzeigen
Private Sub monitor_maske_Click()
  monitor1.Show
End Sub


'neue Kennlinie erzeugen
Private Sub new_profile_Click()
    
  Dim Frage As Variant
  
    
    ''' Ganze Kennlinie vom Speicher löschen...
    ' rrr schräge Bedingung, sollte von was anderem abhängen
   If CWNumEdit1(0).Value > 0 Then
        Frage = MsgBox("Möchten Sie diese Kennlinie abspeichern ?", 36, "Warnung")
        If Frage = 6 Then Call save_profile_Click
    End If
    
    ' ChDir "d:\nlg5_1"   ''''' rrr Zum wegnehmen beim compilieren !!!!!!!!!!
    
    
    datei_laden = CurDir() & "\profiles\null.ini"   'Default Datei
    Call laden_von_ini                              ' vom Ini-File laden
    CmdDelAbschnitt.Enabled = False
    CmdDelAllProfiles.Enabled = False
    abschnitt_zurueck.Enabled = False
    
End Sub


'''''''''''''''''''''''''''''''''''''''''''''
' Maus - Down für die 5 Felder für neue Vorwärts - Bedingungen, werden über Index angesprochen
Private Sub nexte_MouseDown(Index As Integer, Button As Integer, Shift As Integer, x As Single, y As Single)
    sourceidxw = Index
    If Button = 1 Then
       
        nexte(Index).Drag (vbBeginDrag)
            
    End If
    
    If Button = 2 Then
         rechte_source = True
         PopupMenu Edit1
       
    End If
    
    
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Wenn was in die 5 Felder für neue Vorwärts - Bedingungen gezogen wird
Private Sub nexte_DragDrop(Index As Integer, Source As Control, x As Single, y As Single)
    
    Dim uebertragung As kriterium
    Dim temp As Variant
    Dim wtemp As Variant
    Dim einheittemp As Variant
    Dim message As Variant
    
    If Source.Name = "nexte" Then
        temp = nexte(Index).Caption
        wtemp = weiterw(Index).Value
        einheittemp = weitereinheit(sourceidxw)
        
        weiterw(Index).Value = weiterw(sourceidxw).Value
        weiterw(sourceidxw).Value = wtemp
        
        nexte(Index).Caption = Source.Caption
        Source.Caption = temp
        
        weitereinheit(sourceidxw).Caption = weitereinheit(Index).Caption
        weitereinheit(Index).Caption = einheittemp
        
        uebertragung = weiter(aktuelle_abschnitt, sourceidxw)
        weiter(aktuelle_abschnitt, sourceidxw) = weiter(aktuelle_abschnitt, Index)
        weiter(aktuelle_abschnitt, Index) = uebertragung
        
        ' rrr braucht's nicht Debug.Print weiter(aktuelle_abschnitt, weiteranz(aktuelle_abschnitt)).beschreibung
        
    Else
        message = "Diese Bedingung ist hier nicht zugelassen"
        MsgBox (message)
    End If
End Sub


' In der Listbox ein anderes NLG (511, 512,...) gewählt
Private Sub nlg_version_Change()
  MsgBox ("noch nicht implementiert")
End Sub


' Ein Wert in den generellen Daten zu diesem Abschnitt hat geändert
Private Sub num1_ValueChanged(Index As Integer, Value As Variant, PreviousValue As Variant, ByVal OutOfRange As Boolean)
    Select Case Index
        Case 1                                    ' maximaler Ladestrom
          abschnitt_generell(aktuelle_abschnitt).imax = num1(Index).Value
        
        Case 2                                    ' maximale Ladespannung
          abschnitt_generell(aktuelle_abschnitt).umax = num1(Index).Value
        
        Case 3                                    ' Strom zählt für Überladung [%]
          abschnitt_generell(aktuelle_abschnitt).uberladungi = num1(Index).Value
        
        Case 4                                    ' Spannung zählt für Überladung [%]
          abschnitt_generell(aktuelle_abschnitt).uberladungu = num1(Index).Value
        
    End Select
    
End Sub


' Kennlinie zum Gerät schicken
Private Sub Profile_to_Click()
  
  ' MsgBox ("Kennlinie zum Gerät senden: noch nicht implementiert")
  
End Sub


Private Sub Quitter_Click()
     
     If MSComm1.PortOpen = True Then MSComm1.PortOpen = False
     Unload Me
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Doppelklick auf bestehende Vorwärts-Bedingung (eins der 5 möglichen Kästchen) löscht diese
Private Sub nexte_DblClick(Index As Integer)
    Call nexte_loeschen(aktuelle_abschnitt, Index)
End Sub


' Die Eier - Uhr
Private Sub time_out_z_Timer()
    time_out = True
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Doppelklick auf Vorwärts - Bedingungs - Kästchen
Private Sub vorher_DblClick(Index As Integer)
    Call vorher_loeschen(aktuelle_abschnitt, Index)
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Objekt auf Vorwärts - Bedingungs - Kästchen gezogen
Private Sub vorher_DragDrop(Index As Integer, Source As Control, x As Single, y As Single)
   
   Dim uebertragung As kriterium
    Dim temp As Variant
    Dim wtemp As Variant
    Dim einheittemp As Variant
    Dim message As Variant
   
    If Source.Name = "vorher" Then
        temp = vorher(Index).Caption
        wtemp = rueckw(Index).Value
        einheittemp = rueckeinheit(sourceidxv)
        
        rueckw(Index).Value = rueckw(sourceidxv).Value
        rueckw(sourceidxv).Value = wtemp
        
        vorher(Index).Caption = Source.Caption
        Source.Caption = temp
        
        rueckeinheit(sourceidxv).Caption = rueckeinheit(Index).Caption
        rueckeinheit(Index).Caption = einheittemp
        
        
        uebertragung = rueck(aktuelle_abschnitt, sourceidxv)
        rueck(aktuelle_abschnitt, sourceidxv) = rueck(aktuelle_abschnitt, Index)
        rueck(aktuelle_abschnitt, Index) = uebertragung
        
        ' rrr braucht's nicht Debug.Print rueck(aktuelle_abschnitt, rueckanz(aktuelle_abschnitt)).beschreibung
    Else
    message = "Diese Bedingung ist hier nicht zugelassen"
        MsgBox (message)
        
    End If
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Maus - Down für die 5 Felder für neue Rückwärts - Bedingungen, werden über Index angesprochen
Private Sub vorher_MouseDown(Index As Integer, Button As Integer, Shift As Integer, x As Single, y As Single)
    If Button = 1 Then
        sourceidxv = Index
        vorher(Index).Drag (vbBeginDrag)
        
    End If
    
    If Button = 2 Then
        sourceidxv = Index
        rechte_source = False
'        PopupMenu Bearbeitung              ' rrr wozu dient das ???
    End If
       
End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Auf bestimmten Bedingungs - Typ rückwärts geklickt                                                                      '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub BedingTypRW_DblClick()
    
  Dim index1 As Integer
  Dim i As Integer
  
    
    index1 = FnMax(0, (((ComboWechselOption.ListIndex) * 10) + BedingTypRW.ListIndex))
    
    If rueckanz(aktuelle_abschnitt) + 1 < 6 Then          ' Anzahl Bedingungen für Rückwärts < 5 ?
        For i = 1 To rueckanz(aktuelle_abschnitt)         ' alle bestehenden Bedingungen prüfen
            If index1 = rueck(aktuelle_abschnitt, i).beschreibung Then
                MsgBox ("Auswahl nicht mehr möglich" & Chr(13) & "Diese Bedingung ist bereits vorhanden")
                Exit Sub                                  ' Bedingung schon vorhanden, also aussteigen
            
            End If
            
        
        Next i
        
        
        rueckanz(aktuelle_abschnitt) = rueckanz(aktuelle_abschnitt) + 1   ' Eine Rückwärts - Bedingung mehr
        vorher(rueckanz(aktuelle_abschnitt)).Visible = True
        rueckw(rueckanz(aktuelle_abschnitt)).Visible = True
        rueckeinheit(rueckanz(aktuelle_abschnitt)).Visible = True
        
        ' Wenn mehr als eine Bedingung vorhanden, kann man sie verknüpfen
        If rueckanz(aktuelle_abschnitt) > 1 Then CWButton2(rueckanz(aktuelle_abschnitt)).Visible = True
        
        vorher(rueckanz(aktuelle_abschnitt)).Caption = rw(index1, 0)
        rueck(aktuelle_abschnitt, rueckanz(aktuelle_abschnitt)).beschreibung = index1
        rueckw(rueckanz(aktuelle_abschnitt)).Value = Val(rw(index1, 2))
        
        rueckeinheit(rueckanz(aktuelle_abschnitt)).Caption = rw(index1, 4)
        
    Else
        MsgBox ("Keine weiteren Bedingungen möglich")
    End If
    
        
End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Liste 1 und 2 (Auswahlfelder) mit den möglichen Bedingungen füllen                                            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub ListenMitBedingungenFuellen()
    
    Dim idx As Integer
    Dim idx_temp As Integer
    Dim i As Integer
    
    BedingTypRW.Clear                             ' beide Listen von möglichen Bedingungen löschen
    BedingTypVW.Clear
    
    idx = ComboWechselOption.ListIndex      ' Typ aus Combo - Box "Wechsel - Bedingung"
    idx = idx * 10                          ' Index (Zehnerstelle) für Tabellen vw / rw
    idx_temp = idx                          ' Index sichern
    i = 0
    
    Do While vw(idx, 0) <> ""               ' alle Bedingungen dieses Typs in Liste 2 ("nächster Abschn.") füllen
        BedingTypVW.AddItem vw(idx, 0), i
        i = i + 1
        idx = idx + 1
    Loop
    
    idx = idx_temp                          ' Index wiederherstellen
    i = 0
    Do While rw(idx, 0) <> ""               ' alle Bedingungen dieses Typs in Liste 1 ("vorheriger Abschn.") füllen
        BedingTypRW.AddItem rw(idx, 0), i
        i = i + 1
        idx = idx + 1
    Loop
    
    If Not aktuelle_abschnitt = 1 Then       ' beim ersten Abschnitt gibt es nie eine Rückwärts - Bedingung
      If idx_temp = 50 Then                 '
        BedingTypRW.Enabled = False
      Else
        BedingTypRW.Enabled = True
      End If
    End If
    
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' neue Vorwärts - Bedingung erstellen

Private Sub BedingTypVW_DblClick()
    
  Dim index1 As Integer
  Dim i As Integer
  
    
    index1 = FnMax(0, (((ComboWechselOption.ListIndex) * 10) + BedingTypVW.ListIndex))   ' ID der Bedingung, muss > 0 sein (Fehleingaben)
    
    If weiteranz(aktuelle_abschnitt) + 1 <= ANZ_WECHSEL_BEDINGUNGEN Then          ' weitere Bedingung möglich?
        
      For i = 1 To weiteranz(aktuelle_abschnitt)                                  ' Anzahl vorhandene Bedingungen prüfen
        If index1 = weiter(aktuelle_abschnitt, i).beschreibung Then               ' prüfen, ob Bedingung schon ausgewählt
          MsgBox ("Auswahl nicht mehr möglich" & Chr(13) & "Diese Bedingung ist bereits vorhanden")
          Exit Sub
        End If
      Next i
        
      weiteranz(aktuelle_abschnitt) = weiteranz(aktuelle_abschnitt) + 1     ' jetzt eine Bedingung mehr
      nexte(weiteranz(aktuelle_abschnitt)).Visible = True                   ' Felder für neue Bedingung anzeigen
      weiterw(weiteranz(aktuelle_abschnitt)).Visible = True
      weitereinheit(weiteranz(aktuelle_abschnitt)).Visible = True
                                                                            ' wenn mehr als 1 Bedingung, "And/Or" Knopf anzeigen
      If weiteranz(aktuelle_abschnitt) > 1 Then CWButton3(weiteranz(aktuelle_abschnitt)).Visible = True
      nexte(weiteranz(aktuelle_abschnitt)).Caption = vw(index1, 0)          ' passenden Text zur ID der Bedingung anzeigen
      weiter(aktuelle_abschnitt, weiteranz(aktuelle_abschnitt)).beschreibung = index1
      weiterw(weiteranz(aktuelle_abschnitt)).Value = Val(vw(index1, 2))     ' passenden Wert zur ID anzeigen
      weitereinheit(weiteranz(aktuelle_abschnitt)).Caption = vw(index1, 4)  ' passende Einheit zur ID
        
    Else                                                                    ' max. Anzahl Bedingungen erreicht
      MsgBox ("Keine weiteren Bedingungen möglich")
    End If
End Sub



'''' Daten von ini Datei Laden und in Variablen einfügen...

Private Sub laden_von_ini(Optional anfang)
   On Error GoTo ErrHandler
   
    Dim i As Integer
    Dim kn As Integer
    Dim j As Integer
    
    Dim INI_datei As String
    Dim abschnittn As String
    Dim varn As String
    Dim mel1 As String, mel2 As String, meldung As String
    Dim tmp As String
    Dim tmp2() As String
    
    Dim tmp_wecksel As String
  
    Dim tmp_wecksel2() As String
    
    INI_datei = datei_laden
    
    ' Allgemeine Werte von NLG
    
    tmp = ini_lesen(INI_datei, "Brusa Elektronik", "Version")
    nlg_version.ListIndex = Val(Right(tmp, 1))
    
    
    
    tmp = ini_lesen(INI_datei, "Allgemeine Einstellungen", "allgemein")
    tmp2 = Split(tmp, ";")
    
    
    
    For i = 0 To 12
'      nlgx(i) = Val(tmp2(i))
      Call PutGuiValToCapsule(lGuiParArrayAllg(i), CLng(Val(tmp2(i))))  ' bg002 neue Variante: ab in die Datenkapsel damit
    Next i
    
    
    
    If IsMissing(anfang) Then anfang = 1
    For i = anfang To 7
        abschnittn = Trim("Abschnitt" & Trim(Str(i)))
        If i > 1 Then rueckanz(i) = Val(ini_lesen(INI_datei, abschnittn, "anzahlzurueck"))
        If i < 7 Then weiteranz(i) = Val(ini_lesen(INI_datei, abschnittn, "anzahlweiter"))
        
        ''' Generelle Werte des Abschnitts
            tmp = ini_lesen(INI_datei, abschnittn, "generell")
           
            tmp2 = Split(tmp, ";")
               
            abschnitt_generell(i).imax = Val(tmp2(0))
            abschnitt_generell(i).umax = Val(tmp2(1))
            abschnitt_generell(i).uberladungi = Val(tmp2(2))
            abschnitt_generell(i).uberladungu = Val(tmp2(3))
            abschnitt_generell(i).zustandi = tmp2(4)
            abschnitt_generell(i).zustandu = tmp2(5)
            abschnitt_generell(i).zumab1 = tmp2(6)
            
            ' Achtung! Datensatz für LED's doppelt vorhanden: 1 x ".zustandu" / ".zustandi", 1 x "leds"
            For kn = 1 To 3
              leds(i, kn) = Val(Mid(tmp2(4), kn, 1))
              leds(i, kn + 3) = Val(Mid(tmp2(5), kn, 1))
            Next kn
            
            '
           
            '''Weitere Optionen holen
            If i > 1 And rueckanz(i) > 0 Then
                
                For j = 1 To rueckanz(i)
                    tmp_wecksel = ini_lesen(INI_datei, abschnittn, "zurueck" & Trim(Str(j)))
                    
                    tmp_wecksel2 = Split(tmp_wecksel, ";")
                
                    rueck(i, j).beschreibung = Val(tmp_wecksel2(0))
                    rueck(i, j).wert = Val(tmp_wecksel2(1))
                    rueck(i, j).mitnexte = Val(tmp_wecksel2(2))
                Next j
                
            End If
            
            If i < 7 And weiteranz(i) > 0 Then
                
                For j = 1 To weiteranz(i)
                    tmp_wecksel = ini_lesen(INI_datei, abschnittn, "weiter" & Trim(Str(j)))
                    
                    tmp_wecksel2 = Split(tmp_wecksel, ";")
                
                    weiter(i, j).beschreibung = Val(tmp_wecksel2(0))
                    weiter(i, j).wert = Val(tmp_wecksel2(1))
                    weiter(i, j).mitnexte = Val(tmp_wecksel2(2))
                Next j
                
            End If
            
            
    Next i
    Call abschnitt_anzeigen     '
Exit Sub

ErrHandler:
    Call kennlinie_fehler(abschnittn)
    On Error GoTo 0
    
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' bestimmten Abschnitt der Ladekennlinie und ggf. auch den allgemeinen Block anzeigen
Private Sub abschnitt_anzeigen(Optional block)
    
    Dim i As Integer
    Dim test_led As Integer
    Dim sprung As Boolean
    sprung = False
    
    'wenn  "Block" angegeben, dann geh direkt zum Block, sonst mach bei den allgemeinen Einstellungen weiter:
    If IsMissing(block) = False Then
        
      Select Case block
        Case AKTUALISIEREN_RUECKWAERTS
          sprung = True
          GoTo zurueck_anzeige_aktuel
        
        Case AKTUALISIEREN_VORWAERTS
          sprung = True
          GoTo weiter_anzeige_aktuel
                
        Case AKTUALISIEREN_ABSCHNITT
          sprung = False
          GoTo abschnitt_anzeige_aktuel
                
        End Select
            
    End If
    
    ''' Allgemeine Einstellungen anzeigen
        For i = 0 To 11
'            CWNumEdit1(i) = nlgx(i)
'              nlgx(i) = Val(tmp2(i))
          CWNumEdit1(i) = GetGuiValFromCapsule(lGuiParArrayAllg(i))  ' bg002 neue Variante: Daten aus der Datenkapsel holen
        Next i
        
        Reduktions_art.ListIndex = GetGuiValFromCapsule(lGuiParRedArt) ' bg002 statt nlgx(12)
        
    
abschnitt_anzeige_aktuel:       ' Sprung - Label
    
    ''''Abschnitt generell anzeigen
        ''Numerische anzeige
        num1(1).Value = abschnitt_generell(aktuelle_abschnitt).imax
        num1(2).Value = abschnitt_generell(aktuelle_abschnitt).umax
        num1(3).Value = abschnitt_generell(aktuelle_abschnitt).uberladungi
        num1(4).Value = abschnitt_generell(aktuelle_abschnitt).uberladungu
        
        '''Leds anzeige
            For i = 1 To 3
                    
                    'test_led = Val(Mid(abschnitt_generell(aktuelle_abschnitt).zustandi, i, 1))
                    test_led = leds(aktuelle_abschnitt, i)
                    CWButton1(i).Value = False
                    CWButton1(i).OnImages("Image").BlinkInterval = cwSpeedOff
                    
                    Select Case test_led
                        Case 0
                            CWButton1(i).Value = False
                        
                        Case 1
                            CWButton1(i).Value = True
                            CWButton1(i).OnImages("Image").BlinkInterval = cwSpeedOff
                            
                        
                        Case 2
                            CWButton1(i).Value = True
                            CWButton1(i).OnImages("Image").BlinkInterval = cwSpeedFast
                        
                    End Select
                    
                    'test_led = Val(Mid(abschnitt_generell(aktuelle_abschnitt).zustandu, i, 1))
                    test_led = leds(aktuelle_abschnitt, i + 3)
                    CWButton1(i + 3).Value = False
                    CWButton1(i + 3).OnImages("Image").BlinkInterval = cwSpeedOff
                    
                    Select Case test_led
                        Case 0
                            CWButton1(i + 3).Value = False
                        
                        Case 1
                            CWButton1(i + 3).Value = True
                            CWButton1(i + 3).OnImages("Image").BlinkInterval = cwSpeedOff
                            
                        
                        Case 2
                            CWButton1(i + 3).Value = True
                            CWButton1(i + 3).OnImages("Image").BlinkInterval = cwSpeedFast
                        
                    End Select
            Next i  ' Ende Leds
        
        ''' Zum Abschnitt 1 zurück anzeige
            For i = 1 To 4
                test_led = Val(Mid(abschnitt_generell(aktuelle_abschnitt).zumab1, i, 1))
                
                Check1(i).Value = test_led
            Next i
           
        ''' Rückwärts Wechsel Bedingungen anzeigen
zurueck_anzeige_aktuel:       ' Sprung - Label
          For i = 1 To 5
                vorher(i).Visible = False
                rueckw(i).Visible = False
                rueckeinheit(i).Visible = False
                If i > 1 Then CWButton2(i).Visible = False
            Next i
          If rueckanz(aktuelle_abschnitt) > 0 Then
            
            
            For i = 1 To rueckanz(aktuelle_abschnitt)
                vorher(i).Visible = True
                rueckw(i).Visible = True
                rueckeinheit(i).Visible = True
                If i > 1 Then CWButton2(i).Visible = True
                    
                vorher(i).Caption = rw(rueck(aktuelle_abschnitt, i).beschreibung, 0)
                rueckeinheit(i).Caption = rw(rueck(aktuelle_abschnitt, i).beschreibung, 4)
                rueckw(i).Value = rueck(aktuelle_abschnitt, i).wert
                If i > 1 Then CWButton2(i).Value = rueck(aktuelle_abschnitt, i).mitnexte
            Next i
          End If
    If sprung Then Exit Sub
    
    
    ' Bedingungen für Vorwärts-Wechsel optionen anzeigen
weiter_anzeige_aktuel:                                      ' Sprung - Label "aktuelle Vorwärts - Bedingungen"
          For i = 1 To ANZ_WECHSEL_BEDINGUNGEN
            nexte(i).Visible = False                        ' zunächst alle Felder verbergen
            weiterw(i).Visible = False
            weitereinheit(i).Visible = False
            If i > 1 Then CWButton3(i).Visible = False
          Next i
          
          
          If weiteranz(aktuelle_abschnitt) > 0 Then         ' mindestens eine Vorwärts - Bedingung vorhanden?
            
            For i = 1 To weiteranz(aktuelle_abschnitt)
              nexte(i).Visible = True                       ' alle Felder für vorhandene Bedingungen anzeigen
              weiterw(i).Visible = True
              weitereinheit(i).Visible = True
              If i > 1 Then CWButton3(i).Visible = True     ' "And/Or" - Feld anzeigen
                                                            ' Text, Wert und Einheit der Bedingung anzeigen
              nexte(i).Caption = vw(weiter(aktuelle_abschnitt, i).beschreibung, 0)
              weitereinheit(i).Caption = vw(weiter(aktuelle_abschnitt, i).beschreibung, 4)
              weiterw(i).Value = weiter(aktuelle_abschnitt, i).wert
              If i > 1 Then CWButton3(i).Value = weiter(aktuelle_abschnitt, i).mitnexte
            Next i
          End If
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Eins der 5 Felder für neue Rückwärts - Bedingungen löschen, werden über Index angesprochen
Private Sub vorher_loeschen(ab As Integer, idx As Integer)
    
  Dim anz As Integer
  Dim i As Integer
  
  
  
    anz = rueckanz(ab)
    If anz <> idx Then
        For i = idx To anz - 1
            rueck(ab, i) = rueck(ab, i + 1)
        Next i
    End If
    
    rueckanz(ab) = rueckanz(ab) - 1
    Call abschnitt_anzeigen(AKTUALISIEREN_RUECKWAERTS)
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Eins der 5 Felder für neue Vorwärts - Bedingungen löschen, werden über Index angesprochen
Private Sub nexte_loeschen(ab As Integer, idx As Integer)
    
  Dim anz As Integer
  Dim i As Integer
  
    
    anz = weiteranz(ab)
    If anz <> idx Then
        For i = idx To anz - 1
            weiter(ab, i) = weiter(ab, i + 1)
        Next i
    End If
    
    weiteranz(ab) = weiteranz(ab) - 1
    Call abschnitt_anzeigen(AKTUALISIEREN_VORWAERTS)
End Sub


''''''''''''''''''''''''''''''''''''''''''''''

Private Sub loschen_Click()
    Dim sourceidx As Integer
   sourceidx = IIf(rechte_source = True, sourceidxw, sourceidxv)
   If rechte_source Then
        Call nexte_loeschen(aktuelle_abschnitt, sourceidx)
   Else
        Call vorher_loeschen(aktuelle_abschnitt, sourceidx)
   End If
   
   
   
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Fehler in der Kennlinie entdeckt
Private Sub kennlinie_fehler(ab As String)
    
    Dim i As Integer
    Dim meldung As String
    
    meldung = "Ein Fehler ist aufgetreten beim lesen der Kennlinie" & _
            Chr(13) & ab
    For i = 1 To 7
        rueckanz(i) = 0
        weiteranz(i) = 0
    Next i
    
    MsgBox (meldung)
    

End Sub


' Allgemeine Einstellungen von NLG5
' Art der Reduktion bei Erwärmung geändert, also Parameter-Anzeige und Controls anpassen

Private Sub Reduktions_art_Click()

Dim lTmp As Long

  bInvalidCWNumEd1 = True                                   ' während Ctrl Umbau keine Daten speichern
   
  If Reduktions_art.ListIndex = 0 Then                      ' Derating gewählt
    
    lGuiParArrayAllg(GUI_DERAT_HYS) = BATT_TEMP_DERATE      ' ID für Derating Wert in Amp/°C
    lGuiParArrayAllg(GUI_BEZG_HYST) = TEMP_DERATE_NULL      ' ID für Bezugstemperatur für Derating auf 0
    bInvalidCWNumEd1 = True                                 ' solange Ctrl umgebaut wird, keine Daten speichern
    Text1(32).Text = "Batterie-Temperatur-Derating   [Amp./°C]"
    Text1(33).Text = "Bezugstemperatur für Derating auf Null   [°C]"
    CWNumEdit1(GUI_DERAT_HYS).Minimum = -10                 ' Min / Max für Derating
    CWNumEdit1(GUI_DERAT_HYS).Maximum = 0
'    CWNumEdit1(GUI_DERAT_HYS).Value = -4                     ' rrr ???
'    CWNumEdit1(GUI_DERAT_HYS).FormatString = "# ""Amp./°C"   ' anders gelöst
'    CWNumEdit1(GUI_BEZG_HYST).Value = 50                     ' rrr ???
    
  Else                                                      ' Hysterese gewählt
    
    lGuiParArrayAllg(GUI_DERAT_HYS) = BAT_HYSTERES_EIN      ' Temperatur für Ein
    lGuiParArrayAllg(GUI_BEZG_HYST) = BAT_HYSTERES_AUS      ' Temperatur für Aus
    Text1(32).Text = "Einschaltpunkt für Batterietemp.-Hysterese   [°C]"
    Text1(33).Text = "Ausschaltpunkt für Batterietemp.-Hysterese   [°C]"
    CWNumEdit1(GUI_DERAT_HYS).Minimum = 0                   ' Min / Max für Hysterese
    CWNumEdit1(GUI_DERAT_HYS).Maximum = 999
'    CWNumEdit1(GUI_DERAT_HYS).Value = 44                     ' rrr ???
'    CWNumEdit1(GUI_DERAT_HYS).FormatString = "0.0 ""°C"      ' anders gelöst
'    CWNumEdit1(GUI_BEZG_HYST).Value = 45                     ' rrr ???
  
  End If
    
  bInvalidCWNumEd1 = False                                   ' Ctrl ist fertig, Daten wieder speichern
  
  ' Parameter "Reduktions - Art" in Kapsel schreiben
  Call PutGuiValToCapsule(lGuiParRedArt, FnLimit(Reduktions_art.ListIndex, 0, 1))
  
  ' allfällig vorhandene Werte für Derating / Hysterese in Controls laden
  ' "Derating %/°C" oder "Hysterese Ein"
  CWNumEdit1(GUI_DERAT_HYS).Value = GetGuiValFromCapsule(lGuiParArrayAllg(GUI_DERAT_HYS))

  '"Bezugstemperatur" oder "Hysterese Aus"
  CWNumEdit1(GUI_BEZG_HYST).Value = GetGuiValFromCapsule(lGuiParArrayAllg(GUI_BEZG_HYST))
  
End Sub


' für Datei Handling (laden / speichern) Fenstertitel erzeugen und Datei Name ermitteln
Private Function datei(art As Boolean) As String
     
    On Error GoTo keine_datei

    If art Then
        datei_fenster1.DialogTitle = "Save Profile"
        datei_fenster1.ShowSave
        
    Else
        datei_fenster1.DialogTitle = "Load Profile"
        datei_fenster1.ShowOpen
    End If
    
    datei = datei_fenster1.FileName
Exit Function

keine_datei:
        datei = ""
Exit Function

End Function


' Kennlinie aus Datei laden
Private Sub load_profile_Click()
    datei_laden = ""
    datei_laden = datei(False)
    
    If datei_laden = "" Then
        MsgBox ("Keine Datei ausgewählt")
    Else
        
        Call laden_von_ini
        CmdDelAbschnitt.Enabled = True
        CmdDelAllProfiles.Enabled = True
        abschnitt_zurueck.Enabled = True
    End If
    
End Sub


' Kennlinie in Datei speichern
Private Sub save_profile_Click()
      datei_speichern = ""
      datei_speichern = datei(True)
    If datei_speichern = "" Then
        MsgBox ("Keine Datei ausgewählt")
    Else
        Call profile_speichern
    End If
End Sub


'Kennlinie speichern, in Form von Text - Zeilen in einer Ini - Datei
Private Sub profile_speichern()
    ' Beim Schreiben von INI Dateien wird eine Textstream Methode verwendet, um Zeit zu gewinnen (?)
    ' es wird Zeile für Zeile aufbereitet und in die Datei geschrieben
    
    Dim tmp As String
    Dim i As Integer
    Dim j As Integer
    Dim fs As Object              ' genauer: ActiveX Object
    Dim a                         ' "TextStream" Objekt
    
    '"File System Object", aus dem Scripting Päckchen
    Set fs = CreateObject("Scripting.FileSystemObject")
    ' Datei als objekt erstellen
    Set a = fs.CreateTextFile(datei_speichern, True)
    
    '  Allgemeine Daten
    a.writeline ("[Brusa Elektronik]")
    a.writeline ("Geraet=NLG")
    a.writeline ("Version=" & Trim(nlg_version.Text))
    
    a.writeline ("")
    a.writeline ("")
    
    a.writeline ("[Allgemeine Einstellungen]")
    tmp = "allgemein="

    For i = 0 To 11                                         ' die 11 allgemeinen Werte
        tmp = tmp + Trim(Str(CWNumEdit1(i).Value)) + ";"
    Next i
    tmp = tmp + Trim(Str(Reduktions_art.ListIndex))         ' plus die Reduktions - Art (am Schluss)
    a.writeline (tmp)
    
    a.writeline ("")
    a.writeline ("")
    
    
    ' die 7 Ladeabschnitte schreiben
    
    For i = 1 To 7
        tmp = "[Abschnitt" + Trim(Str(i)) + "]"
        a.writeline (tmp)
        
        tmp = "anzahlzurueck=" + Trim(Str(rueckanz(i)))   ' Anzahl rückwärts Bedingungen dieses Abschnitts
        a.writeline (tmp)
        tmp = "anzahlweiter=" + Trim(Str(weiteranz(i)))   ' Anzahl vorwärts Bedingungen dieses Abschnitts
        a.writeline (tmp)
        a.writeline ("")
        
        
        tmp = "generell=" + abschnitt_generell_string(i)  ' generelle Einstellungen dieses Abschnitts schreiben
        a.writeline (tmp)
        a.writeline ("")
        
        If rueckanz(i) > 0 Then
            For j = 1 To rueckanz(i)
                tmp = "zurueck" + Trim(Str(j)) + "=" + zurueck_maske(i, j)
                a.writeline (tmp)
            Next j
           
        End If
        a.writeline ("")
        If weiteranz(i) > 0 Then
            For j = 1 To weiteranz(i)
                tmp = "weiter" + Trim(Str(j)) + "=" + weiter_maske(i, j)
                a.writeline (tmp)
            Next j
           
        End If
        
        a.writeline ("")
        a.writeline ("")
    Next                              ' Ende der 7 Ladeabschnitte
    
    a.Close                           ' Datei schliessen
    
End Sub


' "Generelle Einstellungen"  eines Ladeabschnitts in String packen; "ab" ist der gefragte Abschnitt
Private Function abschnitt_generell_string(ab As Integer) As String

Dim i As Integer

Dim tmp As String
Dim tester As String
Dim string_ganz As String

string_ganz = ""

' For i = 1 To 4
' string_ganz = string_ganz + Trim(Str(num1(i))) + ";"  ' rrr wahrscheinlich falsch! das Control num1 enthält nur die Daten des aktuellen Abschnitts
' Next i

' bg001 Ersatz obiger Schleife, welche wahrscheinlich die Werte für U und I des Abschnitts speichern sollte
string_ganz = string_ganz + Trim(Str(abschnitt_generell(ab).imax)) + ";"          ' Maximalstrom dieses Abschnitts
string_ganz = string_ganz + Trim(Str(abschnitt_generell(ab).umax)) + ";"          ' Maximalspannung dieses Abschnitts
string_ganz = string_ganz + Trim(Str(abschnitt_generell(ab).uberladungi)) + ";"   ' I zählt für Überladung [%]
string_ganz = string_ganz + Trim(Str(abschnitt_generell(ab).uberladungu)) + ";"   ' U zählt für Überladung [%]


tmp = ""
For i = 1 To 6
    tmp = tmp + Trim(Str(leds(ab, i)))
    If i = 3 Then tmp = tmp + ";"
Next i
string_ganz = string_ganz + tmp + ";"

' For i = 1 To 4
' string_ganz = string_ganz + Trim(Str(Check1(i).Value))
' Next i

' bg001 Ersatz obiger Schleife, welche wahrscheinlich die Werte der Checkboxen speichern sollte
string_ganz = string_ganz + abschnitt_generell(ab).zumab1

abschnitt_generell_string = string_ganz
End Function


' String für eine Rückwärts - Bedingung, enthält Beschreibung, Wert und Operator "und/oder"
Private Function zurueck_maske(ab As Integer, anz_nummer As Integer) As String
    Dim tmp
    Dim tester
    
    tmp = tmp + Trim(Str(rueck(ab, anz_nummer).beschreibung)) + ";"
    tmp = tmp + Trim(Str(rueck(ab, anz_nummer).wert)) + ";"
    
    tmp = tmp + IIf(Trim(Str(rueck(ab, anz_nummer).mitnexte)) = "Wahr", "1", "0")
    zurueck_maske = tmp
End Function


' String für eine Vorwärts - Bedingung, enthält Beschreibung, Wert und Operator "und/oder"
Private Function weiter_maske(ab As Integer, anz_nummer As Integer) As String
    Dim tmp
    Dim tester
    
    tmp = tmp + Trim(Str(weiter(ab, anz_nummer).beschreibung)) + ";"
    tmp = tmp + Trim(Str(weiter(ab, anz_nummer).wert)) + ";"
    
    tmp = tmp + IIf(Trim(Str(weiter(ab, anz_nummer).mitnexte)) = "Wahr", "1", "0")
    weiter_maske = tmp
End Function


' String von serieller Schnittstelle einlesen, "out" Zeichen am Anfang abschneiden
Private Function warten_auf_string(out As Integer) As String
    
    Dim ret As Variant
    
    Dim voll As Boolean
    Dim laenge
    Dim zahler As Integer
    voll = False
    inbuffer = ""                                 ' inbuffer leeren
    end_zeichen = False
    time_out_z.Enabled = True
    
    
    Do
        ret = DoEvents()                          ' Windows Ereignisse ausführen ("kooperatives Multitasking")
        
        
        If MSComm1.InBufferCount > 0 Then
            karakter = MSComm1.Input
            inbuffer = inbuffer & karakter
        
        ' Wenn Zeichen ? gefunden, dann String
            If Asc(karakter) = 93 Then
                
                If end_zeichen = False Then
                    end_zeichen = True
                Else
                    end_zeichen = False
                    voll = True
                End If
            
            End If
        
        End If

        
        
        'If time_out = True Then
         '   time_out_z.Enabled = False
          '  time_out = False
            
           ' voll = True
            'MsgBox (inbuffer)
            'inbuffer = ""
            
        'End If
        
    ' bg001 Timeout für Kommunikationsaufbau eingeführt
    Loop Until ((voll = True) Or (time_out = True))
    
    If inbuffer <> "" Then                                ' gültiger String
        laenge = Len(inbuffer)
        inbuffer = Right(inbuffer, laenge - out)          ' echo abschneiden
        laenge = Len(inbuffer)
        inbuffer = Left(inbuffer, laenge - 3)             ''crc und endzeichen abschneiden
    Else                                                  ' Timeout oder ungültiger String
'      MsgBox ("Verbindung konnte nicht hergestellt werden")
    End If
      
    time_out = False                                      ' bg001 allfälliges Timeout reseten..
    time_out_z.Enabled = False                            '..und Timer reseten
    
  warten_auf_string = inbuffer

End Function


' Ladekennlinie von NLG zurücklesen
Private Sub profile_from_Click()
    
    Dim retour As String
    Dim out_string As String
    Dim ret As Variant
    
    
    If MSComm1.PortOpen = False Then
        MsgBox ("Es gibt noch keine Verbindung zum NLG")
        Exit Sub
    Else                                          ' Test-String mir CR / LF
        out_string = ("TestStringVomChargeStar" & Chr(13) & Chr(10))
        MSComm1.Output = out_string
        
        ''' Warten bis alles gesendet ist
        Do
            ret = DoEvents()                      ' Windows Ereignisse ausführen ("kooperatives Multitasking")
        Loop Until MSComm1.OutBufferCount = 0
        
        retour = warten_auf_string(0)
        If retour = "ACK" Then
          If SWITCH_DEBUG = True Then
            MsgBox ("ok, es hat Acknowledge zurück gesendet")
          End If
        End If
           
           
    End If
    
End Sub


' neuen Wert für Vorwärts - Bedingung eingestellt, in Datenstruktur übernehmen
' neu bg001

Private Sub weiterw_ValueChanged(Index As Integer, Value As Variant, PreviousValue As Variant, ByVal OutOfRange As Boolean)
  weiter(aktuelle_abschnitt, Index).wert = Value
End Sub


' neuen Wert für Rückwärts - Bedingung eingestellt, in Datenstruktur übernehmen
' neu bg001

Private Sub rueckw_ValueChanged(Index As Integer, Value As Variant, PreviousValue As Variant, ByVal OutOfRange As Boolean)
  rueck(aktuelle_abschnitt, Index).wert = Value
End Sub


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
  Else                                                  ' Skalierungsfaktor ungültig
    If (SWITCH_DEBUG = True) Then
      MsgBox ("Fehler in PutGuiValToCapsule")
    End If
  End If

End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Wert eines Parameters aus Datenkapsel lesen, für GUI - Control aufbereiten (skalieren)                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function GetGuiValFromCapsule(ByVal lPropID As Long) As Long
  
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
End Function


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' die verschiedenen Parameter mit den zugehörigen Controls des GUI verknüpfen, indem hier die entsprechenden..  '
'..Prop_ID's der Parameter zu den Indizes der Controls geschrieben werden                                       '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub InitPosPropArrays()
  
  ' die Allgemeinen Parameter
  lGuiParArrayAllg(GUI_BATT_NENN) = BATT_NENN_SPANNG
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
  lGuiParRedArt = TEMP_REDUKT_ART_                      ' Temperatur-Reduktions-Art ist in ComboBox dargestellt
  
  '  lGuiParNLGTyp =  'rrr Behandlung NLG Typ noch offen

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
  ' lGuiTypRueck(
  ' lGuiWertRueck
  ' lGuiAndOrRueck
  ' lGuiTypVor
  ' lGuiWertVor
  ' lGuiAndOrVor
  
  
End Sub                           ' Ende InitPosPropArrays



'
Private Sub abschnitt_zurueck_Click()                 ' Wahrscheinlich verwaiste Funktion ohne Bedien - Element
  Call CmdDelAbschnitt_Click                          ' Abschnitt löschen
End Sub


' einen Abschnitt vorwärts hüpfen
Private Sub AbschnVor_Click()
  If aktuelle_abschnitt < ANZ_LADEABSCHNITTE Then     ' vorwärts wechseln, wenn nicht letzter Abschnitt
    aktuelle_abschnitt = aktuelle_abschnitt + 1       ' ein Abschnitt vorwärts
'rrr macht anscheinend schon der Refresh bzw. das ändern des List-Index rrr'        Abschnitt_wechseln    ' neuen Abschnitt anzeigen
    ComboAbschnitt.ListIndex = aktuelle_abschnitt - 1 ' Combo - Box mit den Ladeabschnitten nachführen
    ComboAbschnitt.Refresh                            ' nachgeführte Box neu anzeigen
  End If
End Sub


' einen Abschnitt rückwärts hüpfen
Private Sub AbschnZurueck_Click()
  If aktuelle_abschnitt > 1 Then                      ' rückwärts wechseln, wenn nicht erster Abschnitt
    aktuelle_abschnitt = aktuelle_abschnitt - 1       ' ein Abschnitt zurück
'rrr macht anscheinend schon der Refresh    Abschnitt_wechseln                                ' neuen Abschnitt anzeigen
    ComboAbschnitt.ListIndex = aktuelle_abschnitt - 1 ' Combo - Box mit den Ladeabschnitten nachführen
    ComboAbschnitt.Refresh                            ' nachgeführte Box neu anzeigen
  End If

End Sub



''''''''''''''''''''''''''''''''''''''''''''''
' Vorwärts - Bedingungen löschen
Private Sub CmdDelFwOpt_Click()
   
   Dim Frage As Variant
   
   Frage = MsgBox("Möchten Sie die Wechsel-Bedingungen zum nächsten Abschnitt wirklich löschen? ", 17, "Warnung")
    If Frage = 1 Then
        weiteranz(aktuelle_abschnitt) = 0
        Call abschnitt_anzeigen(AKTUALISIEREN_VORWAERTS)
    End If
    
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Abschnitt löschen
Private Sub CmdDelAbschnitt_Click()
    
   Dim Frage As Variant
    
    Frage = MsgBox("Möchten Sie wirklich diesen Abschnitt auf Standard Werte zurücksetzen ?", 17, "Warnung")
    If Frage = 1 Then Call laden_von_ini(aktuelle_abschnitt)
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
'Ganze Kennlinie löschen
Private Sub CmdDelAllProfiles_Click()
   
   Dim Frage As Variant
    
    Frage = MsgBox("Möchten Sie wirklich die ganze Kennlinie auf Standard Werte zurücksetzen ?", 17, "Warnung")
    If Frage = 1 Then Call laden_von_ini
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Kennlinie laden
Private Sub Command5_Click()
     Call load_profile_Click
        'existiert = datei_exist()
    
End Sub


''''''''''''''''''''''''''''''''''''''''''''''
' Button "löschen aller Rückwärts - Bedingungen"
Private Sub del_all_rw_Click()
    
  Dim Frage As Variant
    
    Frage = MsgBox("Möchten Sie die Wechsel-Bedingungen zum vorherigen Abschnitt wirklich löschen? ", 17, "Warnung")
    If Frage = 1 Then
        rueckanz(aktuelle_abschnitt) = 0
        Call abschnitt_anzeigen(AKTUALISIEREN_RUECKWAERTS)
    End If
    
End Sub


' Verbindung zum NLG abbrechen
Private Sub disconnect_nlg_Click()
    If MSComm1.PortOpen = True Then
      MSComm1.PortOpen = False
      connect_nlg.Enabled = True                    ' bg001 Logik der Verbindungs - Menüs angepasst
      disconnect_nlg.Enabled = False
      
    Else
        MsgBox ("Es besteht keine Verbindung")
    End If
    
End Sub


