VERSION 5.00
Begin VB.Form frmAbout 
   BorderStyle     =   3  'Fester Dialog
   Caption         =   "Über ChargeStar"
   ClientHeight    =   3228
   ClientLeft      =   2340
   ClientTop       =   1932
   ClientWidth     =   4896
   ClipControls    =   0   'False
   Icon            =   "frmAbout.frx":0000
   LinkTopic       =   "Form2"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   2222.58
   ScaleMode       =   0  'Benutzerdefiniert
   ScaleWidth      =   4592.788
   ShowInTaskbar   =   0   'False
   Begin VB.PictureBox picIcon 
      Appearance      =   0  '2D
      AutoSize        =   -1  'True
      BackColor       =   &H80000005&
      ClipControls    =   0   'False
      ForeColor       =   &H80000008&
      Height          =   408
      Left            =   3960
      Picture         =   "frmAbout.frx":0CCA
      ScaleHeight     =   263.118
      ScaleMode       =   0  'Benutzerdefiniert
      ScaleWidth      =   263.118
      TabIndex        =   1
      Top             =   480
      Width           =   408
   End
   Begin VB.CommandButton cmdOK 
      Cancel          =   -1  'True
      Caption         =   "OK"
      Default         =   -1  'True
      Height          =   345
      Left            =   3600
      TabIndex        =   0
      Top             =   2520
      Width           =   1068
   End
   Begin VB.Label lblCopyrightNI 
      Caption         =   $"frmAbout.frx":1994
      BeginProperty Font 
         Name            =   "Small Fonts"
         Size            =   6
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   492
      Left            =   240
      TabIndex        =   7
      Top             =   2640
      Width           =   3132
   End
   Begin VB.Label Label1 
      Alignment       =   2  'Zentriert
      Caption         =   "www.brusa.biz"
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
      Left            =   1320
      TabIndex        =   6
      Top             =   960
      Width           =   1332
   End
   Begin VB.Line Line1 
      BorderColor     =   &H00808080&
      BorderStyle     =   6  'Innen ausgefüllt
      Index           =   1
      X1              =   225.137
      X2              =   4390.165
      Y1              =   1404.604
      Y2              =   1404.604
   End
   Begin VB.Label lblDescription 
      Caption         =   $"frmAbout.frx":1A28
      ForeColor       =   &H00000000&
      Height          =   612
      Left            =   240
      TabIndex        =   2
      Top             =   1320
      Width           =   4572
   End
   Begin VB.Label lblTitle 
      Alignment       =   2  'Zentriert
      Caption         =   "BRUSA ChargeStar NLG 5"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   7.8
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      ForeColor       =   &H00000000&
      Height          =   240
      Left            =   696
      TabIndex        =   4
      Top             =   240
      Width           =   2556
   End
   Begin VB.Line Line1 
      BorderColor     =   &H00FFFFFF&
      BorderWidth     =   2
      Index           =   0
      X1              =   225.137
      X2              =   4390.165
      Y1              =   1404.604
      Y2              =   1404.604
   End
   Begin VB.Label lblVersion 
      Alignment       =   2  'Zentriert
      Caption         =   "Version 1.0"
      Height          =   228
      Left            =   1320
      TabIndex        =   5
      Top             =   600
      Width           =   1332
   End
   Begin VB.Label lblCopyleft 
      Caption         =   "Dieses Programm ist Freeware und darf kostenlos benutzt und verbreitet werden."
      BeginProperty Font 
         Name            =   "Small Fonts"
         Size            =   6
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      ForeColor       =   &H00000000&
      Height          =   348
      Left            =   240
      TabIndex        =   3
      Top             =   2280
      Width           =   3240
   End
End
Attribute VB_Name = "frmAbout"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Name                            frmAbout                                                           '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Beschreibung                    Das About-Fensterchen der NLG - SW, mehr oder weniger vollständig von.. '
'                                       VB (generisch) erstellt. Holt Vieles aus der Windows Registry           '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Was       / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / Nov. '01  / Beat G.     / Anzeige der Version gegenüber Windös-Standard vereinfacht               '
' V2.0      / April '03 / Beat G.     / mehrsprachige Version (Deutsch / Englisch)                              '
'                                                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
'


Option Explicit

' Registrierungsschlüssel-Sicherheitsoptionen...
Const READ_CONTROL = &H20000
Const KEY_QUERY_VALUE = &H1
Const KEY_SET_VALUE = &H2
Const KEY_CREATE_SUB_KEY = &H4
Const KEY_ENUMERATE_SUB_KEYS = &H8
Const KEY_NOTIFY = &H10
Const KEY_CREATE_LINK = &H20
Const KEY_ALL_ACCESS = KEY_QUERY_VALUE + KEY_SET_VALUE + _
                       KEY_CREATE_SUB_KEY + KEY_ENUMERATE_SUB_KEYS + _
                       KEY_NOTIFY + KEY_CREATE_LINK + READ_CONTROL
                     
' Registrierungsschlüssel-Stammtypen...
Const HKEY_LOCAL_MACHINE = &H80000002
Const ERROR_SUCCESS = 0
Const REG_SZ = 1                         ' Null-terminierte Unicode-Zeichenfolge
Const REG_DWORD = 4                      ' 32-Bit-Zahl

Const gREGKEYSYSINFOLOC = "SOFTWARE\Microsoft\Shared Tools Location"
Const gREGVALSYSINFOLOC = "MSINFO"
Const gREGKEYSYSINFO = "SOFTWARE\Microsoft\Shared Tools\MSINFO"
Const gREGVALSYSINFO = "PATH"

Private Declare Function RegOpenKeyEx Lib "advapi32" Alias "RegOpenKeyExA" (ByVal hKey As Long, ByVal lpSubKey As String, ByVal ulOptions As Long, ByVal samDesired As Long, ByRef phkResult As Long) As Long
Private Declare Function RegQueryValueEx Lib "advapi32" Alias "RegQueryValueExA" (ByVal hKey As Long, ByVal lpValueName As String, ByVal lpReserved As Long, ByRef lpType As Long, ByVal lpData As String, ByRef lpcbData As Long) As Long
Private Declare Function RegCloseKey Lib "advapi32" (ByVal hKey As Long) As Long


Private Sub cmdSysInfo_Click()
  Call StartSysInfo
End Sub

Private Sub cmdOK_Click()
  Unload Me
End Sub

' Versions-Darstellung mit "Hauptversion.NebenversionRevision" durch "Hauptversion.Nebenversion.Revision" ersetzt
Private Sub Form_Load()
    Me.Caption = "Info zu " & App.Title
    lblVersion.Caption = "Version " & App.Major & "." & App.Minor & "." & App.Revision
    lblTitle.Caption = App.Title
    lblDescription.Caption = LoadResString(STR_ID_ABOUT_DESCRIPTION + g_lLocaleOffset)
    lblCopyleft.Caption = LoadResString(STR_ID_ABOUT_COPYLEFT + g_lLocaleOffset)
    lblCopyrightNI.Caption = LoadResString(STR_ID_ABOUT_COPYRIGHT_NI + g_lLocaleOffset)
End Sub

Public Sub StartSysInfo()
    On Error GoTo SysInfoErr
  
    Dim rc As Long
    Dim SysInfoPath As String
    
    ' Versuchen, den Systeminfo-Programmpfad/-namen aus der Registrierung abzurufen...
    If GetKeyValue(HKEY_LOCAL_MACHINE, gREGKEYSYSINFO, gREGVALSYSINFO, SysInfoPath) Then
    ' Versuchen, nur den Systeminfo-Programmpfad aus der Registrierung abzurufen...
    ElseIf GetKeyValue(HKEY_LOCAL_MACHINE, gREGKEYSYSINFOLOC, gREGVALSYSINFOLOC, SysInfoPath) Then
        ' Überprüfen, ob bekannte 32-Dateiversion vorhanden ist
        If (Dir(SysInfoPath & "\MSINFO32.EXE") <> "") Then
            SysInfoPath = SysInfoPath & "\MSINFO32.EXE"
            
        ' Fehler - Datei wurde nicht gefunden...
        Else
            GoTo SysInfoErr
        End If
    ' Fehler - Registrierungseintrag wurde nicht gefunden...
    Else
        GoTo SysInfoErr
    End If
    
    Call Shell(SysInfoPath, vbNormalFocus)
    
    Exit Sub

SysInfoErr:
    MsgBox ("Windows System Info Error")
End Sub

Public Function GetKeyValue(KeyRoot As Long, KeyName As String, SubKeyRef As String, ByRef KeyVal As String) As Boolean
    Dim i As Long                                           ' Schleifenzähler
    Dim rc As Long                                          ' Rückgabe-Code
    Dim hKey As Long                                        ' Zugriffsnummer für einen offenen Registrierungsschlüssel
    Dim hDepth As Long                                      '
    Dim KeyValType As Long                                  ' Datentyp eines Registrierungsschlüssels
    Dim tmpVal As String                                    ' Temporärer Speicher eines Registrierungsschlüsselwertes
    Dim KeyValSize As Long                                  ' Größe der Registrierungsschlüsselvariablen
    '------------------------------------------------------------
    ' Registrierungsschlüssel unter KeyRoot {HKEY_LOCAL_MACHINE...} öffnen
    '------------------------------------------------------------
    rc = RegOpenKeyEx(KeyRoot, KeyName, 0, KEY_ALL_ACCESS, hKey) ' Registrierungsschlüssel öffnen
    
    If (rc <> ERROR_SUCCESS) Then GoTo GetKeyError          ' Fehler behandeln...
    
    tmpVal = String$(1024, 0)                             ' Platz für Variable reservieren
    KeyValSize = 1024                                       ' Größe der Variable markieren
    
    '------------------------------------------------------------
    ' Registrierungsschlüsselwert abrufen...
    '------------------------------------------------------------
    rc = RegQueryValueEx(hKey, SubKeyRef, 0, _
                         KeyValType, tmpVal, KeyValSize)    ' Schlüsselwert abrufen/erstellen
                        
    If (rc <> ERROR_SUCCESS) Then GoTo GetKeyError          ' Fehler behandeln
    
    If (Asc(Mid(tmpVal, KeyValSize, 1)) = 0) Then           ' Win95 fügt null-terminierte Zeichenfolge hinzu...
        tmpVal = Left(tmpVal, KeyValSize - 1)               ' Null gefunden, aus Zeichenfolge extrahieren
    Else                                                    ' Keine null-terminierte Zeichenfolge für WinNT...
        tmpVal = Left(tmpVal, KeyValSize)                   ' Null nicht gefunden, nur Zeichenfolge extrahieren
    End If
    '------------------------------------------------------------
    ' Schlüsselwerttyp für Konvertierung bestimmen...
    '------------------------------------------------------------
    Select Case KeyValType                                  ' Datentypen durchsuchen...
    Case REG_SZ                                             ' Zeichenfolge für Registrierungsschlüsseldatentyp
        KeyVal = tmpVal                                     ' Zeichenfolgenwert kopieren
    Case REG_DWORD                                          ' Registrierungsschlüsseldatentyp DWORD
        For i = Len(tmpVal) To 1 Step -1                    ' Jedes Bit konvertieren
            KeyVal = KeyVal + Hex(Asc(Mid(tmpVal, i, 1)))   ' Wert Zeichen für Zeichen erstellen
        Next
        KeyVal = Format$("&h" + KeyVal)                     ' DWORD in Zeichenfolge konvertieren
    End Select
    
    GetKeyValue = True                                      ' Erfolgreiche Ausführung zurückgeben
    rc = RegCloseKey(hKey)                                  ' Registrierungsschlüssel schließen
    Exit Function                                           ' Beenden
    
GetKeyError:      ' Bereinigen, nachdem ein Fehler aufgetreten ist...
    KeyVal = ""                                             ' Rückgabewert auf leere Zeichenfolge setzen
    GetKeyValue = False                                     ' Fehlgeschlagene Ausführung zurückgeben
    rc = RegCloseKey(hKey)                                  ' Registrierungsschlüssel schließen
End Function

