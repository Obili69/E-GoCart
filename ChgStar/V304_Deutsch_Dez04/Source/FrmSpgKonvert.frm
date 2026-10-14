VERSION 5.00
Begin VB.Form FrmSpgKonvert 
   BorderStyle     =   1  'Fest Einfach
   ClientHeight    =   1668
   ClientLeft      =   12
   ClientTop       =   48
   ClientWidth     =   6360
   ControlBox      =   0   'False
   Icon            =   "FrmSpgKonvert.frx":0000
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   1668
   ScaleWidth      =   6360
   StartUpPosition =   3  'Windows-Standard
   Begin VB.CommandButton btnChangeNomVolt 
      Caption         =   "nojo"
      Height          =   372
      Left            =   2520
      TabIndex        =   2
      Top             =   1200
      Width           =   2412
   End
   Begin VB.CommandButton btnCancel 
      Caption         =   "nö"
      Height          =   372
      Left            =   5040
      TabIndex        =   3
      Top             =   1200
      Width           =   1212
   End
   Begin VB.CommandButton btnChangeAll 
      Caption         =   "jo"
      Height          =   372
      Left            =   120
      TabIndex        =   1
      Top             =   1200
      Width           =   2292
   End
   Begin VB.Label lblChangedNomVolt 
      Height          =   732
      Left            =   240
      TabIndex        =   0
      Top             =   240
      Width           =   5892
   End
End
Attribute VB_Name = "FrmSpgKonvert"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Name                            FrmSpgKonvert                                                           '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Modul Beschreibung                    Das Fensterchen fragt nach, ob beim Ändern der Batterie-Nennspannung..  '
'                                       ..alle zugehörigen Werte, die direkt mit der Nennspannung zusammen-..   '
'                                       ..hängen, an den neuen Wert angepasst werden sollen oder nur die..      '
'                                       ..Nennspannung oder gar nix                                             '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Was       / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / Juli '03  / Beat G.     / Für Version 2.0                                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
'

Option Explicit



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fensterchen laden, Titel anzeigen, Info und Button-Texte laden                                                '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_Load()

  Me.Caption = LoadResString(STR_ID_U_CONV_TITLE + g_lLocaleOffset)      ' Titel "Spg konvert" o.ä. laden'
  Me.lblChangedNomVolt.Caption = LoadResString(STR_ID_U_CONV_TEXT + g_lLocaleOffset)      ' Text "wolle anpasse?" laden
  Me.btnCancel.Caption = LoadResString(STR_ID_TXT_CANCEL + g_lLocaleOffset)               ' "abbrechen" text laden
  Me.btnChangeAll.Caption = LoadResString(STR_ID_U_CONV_CHANGE_ALL + g_lLocaleOffset)     ' "alle ändre" text laden
  Me.btnChangeNomVolt.Caption = LoadResString(STR_ID_U_CONV_CHANGE_NOM + g_lLocaleOffset) ' "nom. U ändre" text laden

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Profil nach zugehörigen Werten durchforsten und diese anpassen                                                '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub btnChangeAll_Click()
  
  g_lMsgVoltConversionState = STATE_ADAPT_ALL               ' Alles anpassen
  Unload Me                                                 ' schliessen

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Nur Nennspannung ändern, sonst nix                                                                            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub btnChangeNomVolt_Click()

  g_lMsgVoltConversionState = STATE_CHANGE_NOM_ONLY         ' Nur Nominalspannung ändern
  Unload Me                                                 ' schliessen

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Tu gar nix, abbrechen                                                                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub btnCancel_Click()
  
  Dim iDum1 As Integer, iDum2 As Integer                    ' Dummies
  
  Call Form_QueryUnload(iDum1, iDum2)
  Unload Me                                                 ' schliessen

End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Spannungs-Konvertierungs-Dialog sonstwie geschlossen -> tue dasselbe wie btnCancel_Click, aber lass das..     '
'..Schliessen beiseite, das macht's eh schon selber                                                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub Form_QueryUnload(Cancel As Integer, UnloadMode As Integer)
  
  If (g_lMsgVoltConversionState = STATE_WAIT_FOR_REACTION) Then     ' Fensterli abschiessen ohne definierte Aktion..
    g_lMsgVoltConversionState = STATE_CANCEL                        '..dann auf CANCEL setzen
  End If
  
End Sub


