VERSION 5.00
Begin VB.Form DlgEigenschGeraetBatt 
   BorderStyle     =   1  'Fest Einfach
   Caption         =   " Eigenschaften Gerät / Batterie"
   ClientHeight    =   3696
   ClientLeft      =   2760
   ClientTop       =   3756
   ClientWidth     =   6264
   ControlBox      =   0   'False
   Icon            =   "DlgEigenschGeraetBatt.frx":0000
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   3696
   ScaleWidth      =   6264
   Begin VB.CommandButton PrintButton 
      Caption         =   "Alle Eigenschaften drucken"
      Height          =   375
      Left            =   1680
      TabIndex        =   2
      Top             =   3240
      Width           =   2892
   End
   Begin VB.TextBox CommentsTextBox 
      Height          =   3012
      Left            =   120
      MultiLine       =   -1  'True
      ScrollBars      =   2  'Vertikal
      TabIndex        =   0
      Text            =   "DlgEigenschGeraetBatt.frx":0CCA
      Top             =   120
      Width           =   6012
   End
   Begin VB.CommandButton CancelButton 
      Cancel          =   -1  'True
      Caption         =   "Abbrechen"
      Height          =   375
      Left            =   5040
      TabIndex        =   3
      Top             =   3240
      Width           =   1092
   End
   Begin VB.CommandButton OKButton 
      Caption         =   "OK"
      Height          =   375
      Left            =   120
      TabIndex        =   1
      Top             =   3240
      Width           =   1092
   End
End
Attribute VB_Name = "DlgEigenschGeraetBatt"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
'---------------------------------------------------------------------------------------------------------------'
' Modul Name                          / DlgEigenschGeraetBatt                                                   '
'---------------------------------------------------------------------------------------------------------------'
' Modul Beschreibung                  / Das Fenster, welches Zusatztext/Kommentare anzeigt und druckt.          '
'                                     / Die Kommentare werden mit der Kennlinie abgespeichert, aber nicht in..  '
'                                     /..die Datenkapsel geladen.                                               '
'                                     / Beim Drucken werden alle wichtigen Eigenschaften gedruckt, nicht nur..  '
'                                     /..die im Fenster angezeigten Kommentare                                  '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' Was       / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / Juni '03  / Beat G.     / Für V2.0, Geräte - Fiche komfortabel drucken                            '
' geändert  / 08.11.04  / Beat G.     / Für Version 3.0, bg022, CAN Baudrate und ID's auch drucken              '
' V3.0      / Dez. '05  / Beat G.     / bg027 NLG50x light eingeführt                                           '
'                                                                                                               '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
'                                     /                                                                         '
' PrintButton_Click                   / Kennlinie und Kommentar - Text aufbereiten und drucken                  '
' LinePrintLongText                   / Endlostext in Zeilen zerlegen und drucken                               '
' PrintLine                           / Eine Zeile mit oder ohne Tabs formatieren und drucken                   '
' Form_Load                           / Fensterchen laden, Texte in richtiger Sprache anzeigen etc.             '
' ***_click                           / Eingaben von GUI-Controls behandeln, rufen i.d.R. Dienstfunktionen auf  '
'                                                                                                               '
'---------------------------------------------------------------------------------------------------------------'


Option Explicit

' die "normale" Einstellung für Seite ist A4, die Papier-"Grösse" somit etwa  X: 11900 twips, Y: 16800
' Es muss damit gerechnet werden, dass auch mal auf US-Legal / US Letter gedruckt wird, deshalb wird die..
'..Höhe (also Y) für den Seitenaufbau abgefragt und verrechnet

Const RAND_LINKS            As Long = 800                 ' linker Seitenrand, etwa 15mm
Const RAND_OBEN             As Long = 1000                ' oberer/unterer Seitenrand, etwa 20mm

Const ZEILEN_ABSTAND        As Long = 50                  ' Abstand vor nächster Zeile, etwa 1mm
Const ERSTER_TAB            As Long = 4900                ' erste Tabulator Position, bei ca 1/3 der Breite
Const ZWEITER_TAB           As Long = 9150                ' zweite Tabulator Position, bei ca 4/5 der Breite
' ----------- wenn 3 Tabs (für Kennlinien - Bedingungen) dann: -------------
Const TAB_A                 As Long = 2500                ' erste Tabulator Position
Const TAB_B                 As Long = 6500                ' zweite Tabulator Position
Const TAB_C                 As Long = 9900                ' dritte Tabulator Position
Const TAB_D                 As Long = 10700               ' vierte Tab-Position

Const ZEICHEN_BREITE        As Long = 150                 ' ca. 80 Zeichen / Zeile, also etwa 150 Twips pro Zeichen
Const GANZ_LINIE            As Long = 74                  ' max. Zeichen für ganze Linie
Const LINKE_SPALTE          As Long = 31                  ' max. Zeichen für linke Spalte
Const RECHTE_SPALTE         As Long = 43                  ' max. Zeichen für rechte Spalte




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Kommentar-Text im Fensterchen drucken, plus alles was es sonst noch für die Fiche braucht                     '
' Sehr einfacher Seitenaufbau: A4 hoch, Titel fett, Rest Zeile für Zeile. Keine Anpassung an Papierformate,..   '
'..Schriftgrössen, Druckertypen, max. Zeilenlängen etc.                                                         '
' Das ganze "Formular" wird in dieser Funktion 1:1 aufgebaut                                                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub PrintButton_Click()

Dim strInfo As String             ' Teilstring
Dim strInfo2 As String            ' noch 'n Teilstring
Dim strInfo3 As String            ' noch 'n Teilstring
Dim strInfo4 As String            ' noch 'n Teilstring
Dim strAux As String              ' Hilfsstring
Dim lNLGType As Long              ' Typ des NLG, als Zahl
Dim lOffsetBackup As Long         ' Locale - Einstellung backupen
Dim lTmp As Long                  ' temporärer Long Wert
Dim lTmp2 As Long                 ' noch 'n temporärer Long Wert
Dim lTmp3 As Long                 ' noch 'n temporärer Long Wert
Dim iCount As Integer             ' Schleifenzähler
Dim iCount2 As Integer            ' innerer Schleifenzähler
Dim iAux As Integer               ' Hilfs-Int
Dim fTmp As Single                ' Hilfs-Float
Dim vTmp  As Variant              ' Hilfs-Variant


' debug
'lTmp = Printer.Height
'lTmp2 = Printer.CurrentY
'lTmp3 = Printer.PaperSize
'lTmp = Printer.FontSize
' end debug


If (g_strCommentText <> Me.CommentsTextBox.Text) Then ' Wenn Text geändert, dann..
  g_strCommentText = Me.CommentsTextBox.Text          '..Text aus Textbox neu übernehmen und..
  g_bProfileChanged = True                            '..Abfrage "wollen speichern?"
End If


OKButton.Enabled = False          ' Fenster muss bis zum Abschluss des Print-Jobs offen bleiben, nix fummeln!
CancelButton.Enabled = False
PrintButton.Enabled = False

Printer.CurrentX = RAND_LINKS     ' Seitenränder: von links ca. 15mm, oben ca. 20mm
Printer.CurrentY = RAND_OBEN     ' Seitenrand oben/unten


' ---------------------- Titel -----------------------------------
Printer.FontSize = 20             ' grosser, fetter Titel
Printer.FontBold = True
Printer.Print "BRUSA NLG5 Individual Specifications"

Printer.Print ""                  ' Leerzeile bzw. Abstand


' ---------------------- Kunde usw. -----------------------------------
Printer.FontSize = 11
Printer.FontBold = False

Call PrintLine("Customer:", "Date:           Created:        Checked:")
Printer.Print ""                  ' Leerzeile bzw. Abstand
Printer.Print ""                  ' Leerzeile bzw. Abstand


' ---------------------- Charger Hardware -----------------------------------
Printer.FontSize = 12
Printer.FontBold = True
Printer.CurrentX = RAND_LINKS     ' Seitenrand links

Printer.Print "Charger Hardware"

Printer.FontSize = 11
Printer.FontBold = False

' --- Geräte-Typ (z.B. NLG513) ---
strInfo = "NLG "                      ' NLG-Typ holen und auswerten (X von "NLG 51X")

' bg027 NLG503 hat negativen Offset (NLG510 war alte "Minimal-Nr"), deshalb Fallunterscheidung
lTmp = GetDataFromCapsule(0, INF_NLG_TYP_51X)
If ((lTmp >= 0) And (lTmp <= (NLG_MAX_KLASSE - NLG_MIN_KLASSE))) Then
  lNLGType = NLG_MIN_KLASSE + lTmp   ' "normaler" positiver Offset zu 510 "wie früher"
Else                                 ' negativer Offset zu 510, als unsigned gelesen, ist "grosse Zahl" nahe 2^16
  lNLGType = NLG_MIN_KLASSE + lTmp - MAX_UINT - 1
End If

lNLGType = FnLimit(lNLGType, NLG_LIGHT_MIN_KLASSE, NLG_MAX_KLASSE)
' bg027 alt lNLGType = FnMin((NLG_MIN_KLASSE + (FnLoByteOfLong(GetDataFromCapsule(0, INF_NLG_TYP_51X)))), NLG_MAX_KLASSE)

strInfo = strInfo & CStr(lNLGType)    ' NLG-Typ anzeigen
Call PrintLine("Charger Type: ", strInfo)

' --- Serie-Nr (z.B. 012) ---
strInfo = ""                          ' Serie - Nr LSB holen und auswerten
lTmp = FnMin(GetDataFromCapsule(0, INF_SER_NBR_LSB), MAX_UINT)
lTmp = lTmp + (FnMin(GetDataFromCapsule(0, INF_SER_NBR_MSB), 16) * (MAX_UINT + 1))  ' "Angst-Begrenzung" auf ca. 1 Million
strInfo = strInfo & CStr(lTmp)

Call PrintLine("Serial Number: ", strInfo)

' --- Produktions-Datum (z.B. 31.12.2003) ---
strInfo = ""                          ' Tag hinzufügen
strInfo = strInfo & FormatNumber(FnMin(FnHiByteOfLong(GetDataFromCapsule(0, INF_PROD_DD_MM_)), MAX_DAY), "00") & "."
                                      ' Monat hinzufügen
strInfo = strInfo & Format(FnMin(FnLoByteOfLong(GetDataFromCapsule(0, INF_PROD_DD_MM_)), MAX_MONTH), "00") & "."
                                      ' Jahr hinzufügen
lTmp = FnMin(GetDataFromCapsule(0, INF_PROD_YYYY__), MAX_YEAR)
strInfo = strInfo & CStr(lTmp)
Call PrintLine("Production Date: ", strInfo)

' --- Firmware Version (z.B. 1.10) ---
strInfo = ""                          ' FW-Vers. holen und auswerten
lTmp = FnMin(GetDataFromCapsule(0, INF_FIRMWR_VERS), 9999)
strInfo = strInfo & Format(lTmp / 100, "0.00")             ' Format "x.yy.zz"
lTmp = FnMin(GetDataFromCapsule(0, INF_FIRMWR_REV_), 100)  ' wenn > 99 ist's Müll ->  nicht anzeigen
If (lTmp < 100) Then
  strInfo = strInfo & "." & Format(lTmp, "00")             ' sonst Revisionsindex auch anzeigen
End If
Call PrintLine("Firmware Version: ", strInfo)

' --- Netzspannung ---
strInfo = "100...264 VAC"             ' zulässige Netz - Spannung
Call PrintLine("Mains Input Voltage Range: ", strInfo)

' --- Netzstrom ---
Select Case lNLGType
  Case 501, 503
    strInfo = "12 A"
  Case 511, 512, 513, 514
    strInfo = "16 A"
  Case 521, 522, 523, 524
    strInfo = "32 A"
  Case 531, 532, 533, 534
    strInfo = "48 A"
  Case Else
    strInfo = "unknown"
End Select
Call PrintLine("Max. Mains Current: ", strInfo)

' --- Netzleistung ---
Select Case lNLGType
  Case 501, 503
    strInfo = "1.8 kW"
  Case 511, 512, 513, 514
    strInfo = "3.7 kW"
  Case 521, 522, 523, 524
    strInfo = "7.4 kW"
  Case 531, 532, 533, 534
    strInfo = "11 kW"
  Case Else
    strInfo = "unknown"
End Select
Call PrintLine("Max. Mains Power: ", strInfo)

' --- Ladespannungs-Bereich ---
Select Case lNLGType
  Case 503
    strInfo = "200...520 VDC"
  Case 511, 521, 531
    strInfo = "130...260 VDC"
  Case 512, 522, 532
    strInfo = "180...360 VDC"
  Case 513, 523, 533
    strInfo = "260...520 VDC"
  Case 514, 524, 534
    strInfo = "360...720 VDC"
  Case Else
    strInfo = "unknown"
End Select
Call PrintLine("Charging Voltage Range: ", strInfo)

' --- Ladestrom-Bereich ---
Select Case lNLGType
  Case 503
    strInfo = "8 A"
  Case 511                ' 3.3kW Typen
    strInfo = "25 A"
  Case 512
    strInfo = "18 A"
  Case 513
    strInfo = "12.5 A"
  Case 514
    strInfo = "9 A"
  Case 521                ' 6.7kW Typen
    strInfo = "50 A"
  Case 522
    strInfo = "36 A"
  Case 523
    strInfo = "25 A"
  Case 524
    strInfo = "18 A"
  Case 531                ' 10kW Typen
    strInfo = "75 A"
  Case 532
    strInfo = "54 A"
  Case 533
    strInfo = "37.5 A"
  Case 534
    strInfo = "27 A"
Case Else
    strInfo = "unknown"
End Select
Call PrintLine("Max. Charging Current: ", strInfo)

' --- Ladeleistungs-Bereich ---
Select Case lNLGType
  Case 501, 503
    strInfo = "1.6 kW"
  Case 511, 512, 513, 514
    strInfo = "3.3 kW"
  Case 521, 522, 523, 524
    strInfo = "6.7 kW"
  Case 531, 532, 533, 534
    strInfo = "10 kW"
  Case Else
    strInfo = "unknown"
End Select
Call PrintLine("Max. Charging Power: ", strInfo)


' ---------------------- Geräte-Programmierung -----------------------------------
Printer.FontSize = 12
Printer.FontBold = True
Printer.CurrentX = RAND_LINKS     ' Seitenrand links
Printer.CurrentY = Printer.CurrentY + 4 * ZEILEN_ABSTAND
Printer.Print "Charger Program"

' --- Datei-Name ---
Printer.FontSize = 11
Printer.FontBold = False
If (Len(g_strProfileName) > 0) Then   ' Datei-Name vorhanden?
  strInfo = g_strProfileName          ' Ja, diesen ausgeben
Else
  strInfo = "<none>"                  ' Nein, keine Datei/unbekannt
End If
Call PrintLine("File Name:", strInfo) ' Datei-Name drucken

' --- Betriebs-Art ---
lTmp = GetDataFromCapsule(0, BETR_MASTR_BOOST)
Select Case lTmp
  Case C_STANDARD                 ' Auto
    strInfo = "Automatic"
  Case C_BOOSTER                  ' Booster
    strInfo = "Booster"
  Case C_CAN                      ' CAN
    strInfo = "CAN"
  Case Else                       ' unbekannt
    strInfo = "unknown"
End Select
Call PrintLine("Operation Mode:", strInfo)
If (lTmp = C_BOOSTER) Then        ' für Booster noch die lineare Kennlinie
  strInfo = CStr(GetDataFromCapsule(0, EXT_STROM_P_MIN_) / GetScaleOfPar(0, EXT_STROM_P_MIN_)) & " A"
  Call PrintLine("Ext. Current for OFF:", strInfo)
  strInfo = CStr(GetDataFromCapsule(0, EXT_STROM_P_MAX_) / GetScaleOfPar(0, EXT_STROM_P_MAX_)) & " A"
  Call PrintLine("Ext. Current for Full Power:", strInfo)
End If

' --- Batterie-Nennspannung ---
strInfo = CStr(GetDataFromCapsule(0, BATT_NENN_SPANNG) / GetScaleOfPar(0, BATT_NENN_SPANNG)) & " V"
Call PrintLine("Nominal Battery Voltage:", strInfo)

' --- Maximaler Netzstrom ---
lTmp = GetDataFromCapsule(0, BETR_MASTR_BOOST)
If lTmp = C_CAN Then              ' bei CAN nicht einstellbar
  strInfo = "N/A"
Else                              ' sonst immer
  strInfo = CStr(GetDataFromCapsule(0, MAX_NETZ_STROM__) / GetScaleOfPar(0, MAX_NETZ_STROM__)) & " A"
End If
Call PrintLine("Maximum Mains Current:", strInfo)

' --- Ventilation Request vom Control Pilot ---
lTmp = GetDataFromCapsule(0, VENTI_REQ_BIT_CP) / GetScaleOfPar(0, VENTI_REQ_BIT_CP)
If (lTmp = BTRUE) Then
  Call PrintLine("Control Pilot:", "Ventilation Request ON")
End If

' ------------- Batterie-Temperatur-Behandlung ----------------
Printer.FontSize = 11
Printer.FontBold = True
Printer.CurrentX = RAND_LINKS     ' Seitenrand links
Printer.CurrentY = Printer.CurrentY + 1.5 * ZEILEN_ABSTAND
Printer.Print "Battery Temperature Handling"
Printer.FontBold = False

' --- welche Batt. Temp. Sensoren 1-3 aktiv ---
lTmp = FnMin(GetDataFromCapsule(0, AKTIVE_TEMP_SENS), 111)
For iAux = 0 To 2
  If (FnGetCharOfNumber(lTmp, 2 - iAux) > 0) Then ' dieser Sensor ist aktiv
    strInfo = "NTC 33kOhm/25°C, Pin " & CStr(20 + iAux) & " to 8, must be connected!"
  Else                                          ' dieser Sensor ist nicht aktiv
    strInfo = "not active"
  End If
Call PrintLine(("Batt. Temp. Sensor #" & CStr(iAux + 1) & ": "), strInfo)
Next iAux

' --- Temp. Koeff. Gasungsspannung ---
lTmp = FnMin(GetDataFromCapsule(0, AKTIVE_TEMP_SENS), 111)
If (lTmp > 0) Then                ' mindestens ein Sensor aktiv
fTmp = (GetDataFromCapsule(0, TEMP_KOEF_GAS_SP) / GetScaleOfPar(0, TEMP_KOEF_GAS_SP))
  strInfo = CStr(fTmp) & " %/°C"
Else                              ' kein Sensor aktiv
  strInfo = "N/A"
End If
Call PrintLine("Temp. Coeff. Gassing Voltage:", strInfo)

' --- Referenz-Temperatur für Spannungen ---
lTmp = FnMin(GetDataFromCapsule(0, AKTIVE_TEMP_SENS), 111)
If (lTmp > 0) Then                ' mindestens ein Sensor aktiv
fTmp = (GetDataFromCapsule(0, BEZUGS_TEMP_SPNG) / GetScaleOfPar(0, BEZUGS_TEMP_SPNG))
  strInfo = CStr(fTmp) & " °C"
Else                              ' kein Sensor aktiv
  strInfo = "N/A"
End If
Call PrintLine("Reference Temp. for Voltages:", strInfo)

' --- Derating oder Hysterese ---
lTmp = FnMin(GetDataFromCapsule(0, AKTIVE_TEMP_SENS), 111)
If (lTmp > 0) Then                ' mindestens ein Sensor aktiv
  lTmp = GetDataFromCapsule(0, TEMP_REDUKT_ART_)
  If (lTmp = C_DERATING) Then       ' Temp. Reduktionsart ist Derating, also entsprechend darstellen
    strInfo = CStr(GetDataFromCapsule(0, BATT_TEMP_DERATE) / GetScaleOfPar(0, BATT_TEMP_DERATE)) & " A/°C,  0 A at " & CStr(GetDataFromCapsule(0, TEMP_DERATE_NULL) / GetScaleOfPar(0, TEMP_DERATE_NULL)) & " °C"
    Call PrintLine("Battery Temp. Derating:", strInfo)
  Else                              ' Hysterese, anders darstellen
    strInfo = "ON below " & CStr(GetDataFromCapsule(0, BAT_HYSTERES_EIN) / GetScaleOfPar(0, BAT_HYSTERES_EIN)) & " °C,  OFF above " & CStr(GetDataFromCapsule(0, BAT_HYSTERES_AUS) / GetScaleOfPar(0, BAT_HYSTERES_AUS)) & " °C"
    Call PrintLine("Batt. Temp. Hysteresis:", strInfo)
  End If
Else
  Call PrintLine("Current Reduction:", "N/A")
End If

' ------------- Abschalten in jedem Fall ----------------
Printer.FontBold = True
Printer.CurrentX = RAND_LINKS     ' Seitenrand links
Printer.CurrentY = Printer.CurrentY + 1.5 * ZEILEN_ABSTAND
Printer.Print "Emergency Shutdown Limits"
Printer.FontBold = False

' --- Batterie Übertemperatur ---
lTmp = FnMin(GetDataFromCapsule(0, AKTIVE_TEMP_SENS), 111)
If (lTmp > 0) Then       ' nur wenn Temp.Sensor angeschlossen
  strInfo = CStr((GetDataFromCapsule(0, BATT_TEMP_UEBER_)) / GetScaleOfPar(0, BATT_TEMP_UEBER_)) & " °C"
Else
  strInfo = "N/A"
End If
Call PrintLine("Max. Battery Temperature:", strInfo)

' --- Batterie Überspannung ---
strInfo = CStr((GetDataFromCapsule(0, BAT_SPANNG_UEBER)) / GetScaleOfPar(0, BAT_SPANNG_UEBER)) & " V"
Call PrintLine("Max. Battery Voltage:", strInfo)

' --- Batterie Überladung ---
strInfo = CStr((GetDataFromCapsule(0, BAT_LAD_MEHR_ALS)) / GetScaleOfPar(0, BAT_LAD_MEHR_ALS)) & " Ah"
Call PrintLine("Max. Battery Charge:", strInfo)

' --- Ladezeit überschritten ---
strInfo = CStr((GetDataFromCapsule(0, LAD_ZEIT_MER_ALS)) / GetScaleOfPar(0, LAD_ZEIT_MER_ALS)) & " h"
Call PrintLine("Max. Charging Time:", strInfo)



' ------------- Eigentliche Kennlinie / Lade-Abschnitte ----------------

lTmp = GetDataFromCapsule(0, BETR_MASTR_BOOST)  ' ----------------  Auto/Booster/CAN ??? --------------
If lTmp = C_STANDARD Then           ' bei Auto die Kennlinie drucken, bei CAN und Booster keine Kennlinie
 
  lOffsetBackup = g_lLocaleOffset   ' Spracheinstellung backupen,..
  g_lLocaleOffset = LOCALE_ENGLISH  '..für Ausdruck temporär auf Englisch setzen
  
  Printer.FontBold = True
  Printer.CurrentX = RAND_LINKS     ' Seitenrand links
  Printer.CurrentY = Printer.CurrentY + 1.5 * ZEILEN_ABSTAND
  Printer.Print "Charging Profile"
  Printer.FontBold = False
  
  ' --- Lade-Abschnitte ---
  iCount = 0                            ' äusserer Schleifenzähler, für Abschnitts-Nummer
  Do                                    ' alle "bevölkerten" Abschnitte durchzählen
                                        
    If ((Printer.Height - Printer.CurrentY) < (2.5 * RAND_OBEN)) Then ' wenn aktuelle Y-Position zu weit unten für ganzen..
      Printer.NewPage                   '..Abschnitt, dann mach 'n Seitenvorschub und setz das Y wieder oben links
      Printer.CurrentX = RAND_LINKS     ' Seitenrand links
      Printer.CurrentY = RAND_OBEN      ' Seitenrand oben
    End If
    
    lTmp3 = ANZ_VORWAE_BED + (iCount * LAENGE_ABSCHN)     ' ganze Prop_ID berechnen
    lTmp = GetDataFromCapsule(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3)) ' Anz. Vorwärts-Bedingungen des akt. Abschnitts lesen
    
    For iCount2 = 0 To lTmp             ' alle Vorwärts-Bedingungen des aktuellen Abschnitts durchzählen
      If (iCount2 = 0) Then             ' 1. Zeile: links Abschnitts-Bezeichnung, rechts max. I / max. U  / ggf. until
        Printer.CurrentY = Printer.CurrentY + ZEILEN_ABSTAND
        strAux = "Section " & iCount + 1 & ":"  ' Abschnitt Nr und max. Strom / max. Spg im Abschnitt
        lTmp3 = MAX_I_PH_STROM + (iCount * LAENGE_ABSCHN) ' ganze Prop_ID berechnen
        fTmp = (GetDataFromCapsule(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3)) / GetScaleOfPar(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3)))
        strInfo = "max. Current [A]: " & CStr(fTmp)       ' Strom aus Datenkapsel holen und anzeigen
        lTmp3 = MAX_U_PH_SPANN + (iCount * LAENGE_ABSCHN) ' ganze Prop_ID berechnen
        fTmp = (GetDataFromCapsule(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3)) / GetScaleOfPar(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3)))
        strInfo2 = "max. Voltage [V]: " & CStr(fTmp)      ' Spannung aus Datenkapsel holen und anzeigen
        If (lTmp > 0) Then
          strInfo3 = "until"
        Else
          strInfo3 = " "
        End If
        strInfo4 = ""
      Else
        strAux = " "                    ' 2. und weitere Zeilen: rechts Bedingung, Wert, Und/Oder
        lTmp3 = TYP_VORW_BED_1 + ((iCount * LAENGE_ABSCHN) + (3 * (iCount2 - 1))) ' ganze Prop_ID berechnen
        lTmp2 = GetDataFromCapsule(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3))  ' Bedingungstyp als Zahl lesen
        strInfo = "-" & GetTextOfCondID(lTmp2, BTRUE, g_lLocaleOffset)    ' Typ / Text der Bedingung, links schreiben
        lTmp3 = VAL_VORW_BED_1 + ((iCount * LAENGE_ABSCHN) + (3 * (iCount2 - 1))) ' ganze Prop_ID berechnen
        fTmp = (GetDataFromCapsule(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3)) / GetScaleOfPar(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3)))
        strInfo2 = ""
        strInfo3 = CStr(fTmp)                     ' Wert der Bedingung, weit rechts schreiben
        If (iCount2 < lTmp) Then                  ' kommt noch eine nächste Bedingung?
          lTmp3 = AND_OR_VORW_12 + ((iCount * LAENGE_ABSCHN) + (3 * (iCount2 - 1))) ' ganze Prop_ID berechnen
          lTmp2 = GetDataFromCapsule(FnHiByteOfLong(lTmp3), FnLoByteOfLong(lTmp3))  ' Wert für Und/Oder aus Kapsel holen
          If (lTmp2 = C_OR) Then                  ' Und / Oder Wert holen und anzeigen
            strInfo4 = LoadResString(STR_ID_PRF_OR + g_lLocaleOffset)         ' ODER
          Else
            strInfo4 = LoadResString(STR_ID_PRF_AND + g_lLocaleOffset)        ' UND
          End If
        Else                                      ' keine weitere Bedingung
          strInfo4 = " "
        End If
      End If
      Call PrintLine(strAux, strInfo, strInfo2, strInfo3, strInfo4)
    Next iCount2                        ' nächste Bedingung, falls vorhanden
    iCount = iCount + 1                 ' nächster Abschnitt
  Loop While ((lTmp > 0) And (iCount < ANZ_LADEABSCHNITTE))
                                        ' Ende Ladeabschnitte
  g_lLocaleOffset = lOffsetBackup       ' wieder auf Originalsprache setzen
  
End If

' --------------- Ende eigentliche Lade-Kennlinie
  

' ------------- CAN Parameter, wenn CAN-Mode ----------------

lTmp = GetDataFromCapsule(0, BETR_MASTR_BOOST)  ' ----------------  Auto/Booster/CAN ??? --------------
If lTmp = C_CAN Then                ' bei CAN die CAN-Parameter drucken
  
  Printer.FontBold = True           ' fetter Titel "CAN Parameters"
  Printer.CurrentX = RAND_LINKS     ' Seitenrand links
  Printer.CurrentY = Printer.CurrentY + 1.5 * ZEILEN_ABSTAND
  Printer.Print "CAN Parameters"
  Printer.FontBold = False

  ' --- CAN Baud Rate ---
  lTmp = GetDataFromCapsule(0, CAN_BAUD_RATE___) / GetScaleOfPar(0, CAN_BAUD_RATE___)
  Select Case (lTmp)
  Case CAN_BAUDRATE_1M
    strInfo = "1 MBit / sec"
  Case CAN_BAUDRATE_500k
    strInfo = "500 kBit / sec"
  Case CAN_BAUDRATE_250k
    strInfo = "250 kBit / sec"
  Case CAN_BAUDRATE_125k
    strInfo = "125 kBit / sec"
  Case Else
    strInfo = "unknown"
  End Select
  Call PrintLine("Baud Rate:", strInfo)

  ' --- RX/TX Msg Identifiers ---
  strInfo = "0x" & Format(Hex((GetDataFromCapsule(0, CAN_RX_ID_CTL___) / GetScaleOfPar(0, CAN_RX_ID_CTL___))), "000")
  Call PrintLine("Rx Msg  'NLG5_CTL'  ID (hex):", strInfo)
  
  strInfo = "0x" & Format(Hex((GetDataFromCapsule(0, CAN_TX_ID_ST____) / GetScaleOfPar(0, CAN_TX_ID_ST____))), "000")
  Call PrintLine("Tx Msg  'NLG5_ST'  ID (hex):", strInfo)
  strInfo = "0x" & Format(Hex((GetDataFromCapsule(0, CAN_TX_ID_ACT_I_) / GetScaleOfPar(0, CAN_TX_ID_ACT_I_))), "000")
  Call PrintLine("Tx Msg  'NLG5_ACT_I'  ID (hex):", strInfo)
  strInfo = "0x" & Format(Hex((GetDataFromCapsule(0, CAN_TX_ID_ACT_II) / GetScaleOfPar(0, CAN_TX_ID_ACT_II))), "000")
  Call PrintLine("Tx Msg 'NLG5_ACT_II'  ID (hex):", strInfo)
  strInfo = "0x" & Format(Hex((GetDataFromCapsule(0, CAN_TX_ID_TEMP__) / GetScaleOfPar(0, CAN_TX_ID_TEMP__))), "000")
  Call PrintLine("Tx Msg 'NLG5_TEMP'  ID (hex):", strInfo)
  strInfo = "0x" & Format(Hex((GetDataFromCapsule(0, CAN_TX_ID_ERR___) / GetScaleOfPar(0, CAN_TX_ID_ERR___))), "000")
  Call PrintLine("Tx Msg 'NLG5_ERR'  ID (hex):", strInfo)

End If

' -------------- Ende CAN Parameter



' ---------------------- Kommentar / freier Zusatztext -----------------------------------

If (g_strCommentText <> "") Then

  ' --- Titel "Notes" ---
  If ((Printer.Height - Printer.CurrentY) < (1.5 * RAND_OBEN)) Then ' wenn aktuelle Y-Position zu weit unten,..
    Printer.NewPage                   '.. dann mach 'n Seitenvorschub und setz das Y wieder oben links
    Printer.CurrentX = RAND_LINKS     ' Seitenrand links
    Printer.CurrentY = RAND_OBEN      ' Seitenrand oben
  End If
  
  Printer.FontSize = 12
  Printer.FontBold = True
  Printer.CurrentX = RAND_LINKS     ' Seitenrand links
  Printer.CurrentY = Printer.CurrentY + 4 * ZEILEN_ABSTAND
  Printer.Print "Notes"
  
  ' --- Datei-Name ---
  Printer.FontSize = 11
  Printer.FontBold = False
  Printer.CurrentY = Printer.CurrentY + 0.5 * ZEILEN_ABSTAND
  
  
  Call LinePrintLongText(g_strCommentText)

End If

Printer.EndDoc                    ' Drucken lassen

OKButton.Enabled = True           ' Print-Job fertig, jetz darfsch wieder fummeln!
CancelButton.Enabled = True
PrintButton.Enabled = True

End Sub               ' PrintButton_Click



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Einen "Endlostext" mit oder ohne CR/LF/TAB sinnvoll in einzelne Zeilen zerlegen und drucken                   '
' Das heisst, in zweispaltigen, max. 80-Zeichen Text umbrechen                                                  '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub LinePrintLongText(strInText As String)

Dim lCount As Long                                    ' Zählvariable
Dim lAnzZeichen As Long                               ' Tatsächliche Anzahl zeichen in dieser Spalte
Dim lMaxAnzZeichen As Long                            ' Maximale Anzahl zeichen in dieser Spalte
Dim bTab As Long                                      ' Tab (rechte Spalte) or not Tab (linke Spalte)?
Dim strLeft As String                                 ' Text links
Dim strRight As String                                ' Text rechts


strLeft = ""                                          ' leerer String zum Anfang
strRight = ""                                         ' leerer String zum Anfang
lCount = 1                                            ' Zähler auf Start
lMaxAnzZeichen = 80                                   ' wenn einspaltig max. 80 Zeichen
bTab = BFALSE                                         ' solange kein Tab gefunden: linke Spalte

While (lCount <= Len(strInText))                      ' Text-Ende noch nicht erreicht

  If ((Printer.Height - Printer.CurrentY) < (1.2 * RAND_OBEN)) Then ' wenn aktuelle Y-Position zu weit unten für ganzen..
    Printer.NewPage                   '..Abschnitt, dann mach 'n Seitenvorschub und setz das Y wieder oben links
    Printer.CurrentX = RAND_LINKS     ' Seitenrand links
    Printer.CurrentY = RAND_OBEN      ' Seitenrand oben
  End If

  
  If (Mid(strInText, lCount, 2) = vbCrLf) Then        ' Zeilenvorschub im Text gefunden
    Call PrintLine(strLeft, strRight)                 ' Zeile drucken
    strLeft = ""                                      ' nach dem Drucken linke Spalte leeren
    strRight = ""                                     ' nach dem Drucken rechte Spalte leeren
    lAnzZeichen = 0                                   ' wieder Platz für neue Linie
    bTab = BFALSE                                     ' Text in linke Spalte
    lCount = lCount + 1
    lMaxAnzZeichen = GANZ_LINIE                       ' wenn einspaltig max. ca. 80 Zeichen
  Else
    If (lAnzZeichen > lMaxAnzZeichen) Then            ' Zeile zu lang, also Umbruch einfügen
                                                      ' es folgt kein Leerzeichen, es führt auch keines, also Bindestrichli anhängen
      If (Mid(strInText, lCount, 1) <> " ") And (Mid(strInText, lCount - 1, 1) <> " ") Then
        If (bTab = BTRUE) Then                        ' rechte Spalte?
          strRight = strRight & "-"                   ' Bindestrichli
        Else                                          ' linke Spalte
          strLeft = strLeft & "-"                     ' Bindestrichli
        End If
        Call PrintLine(strLeft, strRight)             ' Zeile drucken
        strLeft = ""                                  ' nach dem Drucken linke Spalte leeren
        strRight = ""                                 ' nach dem Drucken rechte Spalte leeren
        strLeft = strLeft & Mid(strInText, lCount, 1) ' zeichen kopieren
        lAnzZeichen = 1                               ' 1 Zeichen auf neuer Linie
      Else                                            ' führendes oder folgendes Leerzeichen?
        Call PrintLine(strLeft, strRight)             ' Zeile drucken
        strLeft = ""                                  ' nach dem Drucken linke Spalte leeren
        strRight = ""                                 ' nach dem Drucken rechte Spalte leeren
        If ((Mid(strInText, lCount, 1) <> " ")) Then  ' kein führendes Leerzeichen
          strLeft = strLeft & Mid(strInText, lCount, 1) ' zeichen kopieren
          lAnzZeichen = 0                             ' führendes Leerzeichen abschneiden, 0 Zeichen auf neuer Linie
        Else
          lAnzZeichen = 0                             ' führendes Leerzeichen abschneiden, 0 Zeichen auf neuer Linie
        End If
      End If
      bTab = BFALSE                                   ' Text in linke Spalte
      lMaxAnzZeichen = GANZ_LINIE                     ' wenn einspaltig max. ca. 80 Zeichen
    Else
      If (Mid(strInText, lCount, 1) = vbTab) Then     ' Tabulator gefunden
        If (bTab = BTRUE) Then                        ' schon in rechter Spalte, nix mehr Tabulator
          lAnzZeichen = lAnzZeichen + 1               ' ein Zeichen mehr
          strRight = strRight & " "                   ' Plus ein Leerzeichen in rechten Spaltentext
          lMaxAnzZeichen = RECHTE_SPALTE              ' max. ca. 45 Zeichen in rechter Spalte bei zweispaltiger Darstellung
        Else
          lMaxAnzZeichen = LINKE_SPALTE               ' max. ca. 30 Zeichen in linker Spalte bei zweispaltiger Darstellung
        End If
        While (Mid(strInText, lCount + 1, 1) = vbTab)
          lCount = lCount + 1                         ' mehrfach-Tabs überspringen
        Wend
        If (lAnzZeichen > lMaxAnzZeichen) Then        ' wenn schon zuviele Zeichen in linker Spalte, dann Umbruch einfügen
          Call PrintLine(strLeft, strRight)           ' Zeile drucken
          strLeft = ""                                ' nach dem Drucken linke Spalte leeren
          strRight = ""                               ' nach dem Drucken rechte Spalte leeren
          bTab = BFALSE                               ' Text in linke Spalte
          lMaxAnzZeichen = GANZ_LINIE                 ' max. ca. 80 Zeichen bei (nun wieder) einspaltiger Darstellung
          lAnzZeichen = 0                             ' neu beginnen in linker Spalte
        Else
          If (bTab = False) Then                      ' noch nicht in rechter Spalte gewesen
            bTab = BTRUE                              ' ab jetzt in rechten String schreiben
            lAnzZeichen = 0                           ' neu beginnen in rechter Spalte
          End If
          lMaxAnzZeichen = RECHTE_SPALTE              ' max. ca. 45 Zeichen in rechter Spalte
        End If
      Else
        If (bTab = BTRUE) Then
          strRight = strRight & Mid(strInText, lCount, 1) ' zeichen in rechten Spaltentext kopieren
          lAnzZeichen = lAnzZeichen + 1                   ' 1 Zeichen mehr
        Else
          strLeft = strLeft & Mid(strInText, lCount, 1)   ' zeichen in linken Spaltentext kopieren
          lAnzZeichen = lAnzZeichen + 1                   ' 1 Zeichen mehr
        End If
      End If
    End If
  End If
  lCount = lCount + 1                                 ' nächstes Zeichen
Wend

Call PrintLine(strLeft, strRight)                 ' Rest drucken

End Sub               ' LinePrintLongText




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Eine Zeile an den Printer liefern, wobei max. 3 (bzw.4) Strings an bestimmten Tab-Pos. platziert werden können'
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub PrintLine(strLeftString As String, Optional strMidString As String, Optional strRightString As String, Optional strFarRight As String, Optional strUltraRight As String)

Dim strTemp As String                   ' String zum Text basteln
Dim lCurrentY As Long                   ' aktuelle Zeilenposition

  strTemp = strLeftString               ' zunächst mal der linke String, muss immer was drin sein
  Printer.CurrentX = RAND_LINKS         ' Seitenrand links
  Printer.CurrentY = Printer.CurrentY + ZEILEN_ABSTAND
  lCurrentY = Printer.CurrentY          ' aktuelle Zeilenpos sichern, wegen Zeilenvorschub
  Printer.Print strLeftString
  
  If (strFarRight <> "") Then             ' Murks: nachträglich eingeführter 4.Tab, soll nicht alles bisherige ändern
  ' btw: Sch..VB kann wieder mal nicht zwischen "leerem String" und "kein String" unterscheiden, deshalb geht das "is missing" hier nicht
    If Not IsMissing(strMidString) Then   ' mittleren String drucken, falls vorhanden
      Printer.CurrentX = TAB_A            ' erste Tabulator Position
      Printer.CurrentY = lCurrentY        ' gleiche Zeilenposition wie linker String
      Printer.Print strMidString          '
    End If
  
    If Not IsMissing(strRightString) Then ' rechten String drucken, falls vorhanden
      Printer.CurrentX = TAB_B            ' zweite Tabulator Position
      Printer.CurrentY = lCurrentY        ' gleiche Zeilenposition wie linker String
      Printer.Print strRightString        '
    End If
    
    Printer.CurrentX = TAB_C              ' dritte Tabulator Position
    Printer.CurrentY = lCurrentY          ' gleiche Zeilenposition wie linker String
    Printer.Print strFarRight          '
 
    If Not IsMissing(strMidString) Then   ' mittleren String drucken, falls vorhanden
      Printer.CurrentX = TAB_D            ' erste Tabulator Position
      Printer.CurrentY = lCurrentY        ' gleiche Zeilenposition wie linker String
      Printer.Print strUltraRight          '
    End If
  
  Else                                    ' "Normalfall", frühere Version
    
    If Not IsMissing(strMidString) Then   ' mittleren String drucken, falls vorhanden
      Printer.CurrentX = ERSTER_TAB       ' erste Tabulator Position
      Printer.CurrentY = lCurrentY        ' gleiche Zeilenposition wie linker String
      Printer.Print strMidString          '
    End If
  
    If Not IsMissing(strRightString) Then ' rechten String drucken, falls vorhanden
      Printer.CurrentX = ZWEITER_TAB      ' zweite Tabulator Position
      Printer.CurrentY = lCurrentY        ' gleiche Zeilenposition wie linker String
      Printer.Print strRightString          '
    End If
    
  End If


End Sub                   ' PrintLine



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fensterchen laden, Kommentar-Text anzeigen                                                                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


Private Sub Form_Load()
 
  DlgEigenschGeraetBatt.Caption = LoadResString(STR_ID_PROPERTIES_TITLE + g_lLocaleOffset)          ' Titel laden
  DlgEigenschGeraetBatt.CancelButton.Caption = LoadResString(STR_ID_TXT_CANCEL + g_lLocaleOffset)   ' Cancel-Button Text laden
  DlgEigenschGeraetBatt.PrintButton.Caption = LoadResString(STR_ID_PROP_PRINT_ALL + g_lLocaleOffset)   ' Cancel-Button Text laden
  
  If (g_bConnectionEstablished = True) Then
    DlgEigenschGeraetBatt.PrintButton.Enabled = True
  Else
    DlgEigenschGeraetBatt.PrintButton.Enabled = False
  End If
  
  Me.CommentsTextBox.Text = g_strCommentText

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fensterchen schliessen, ohne etwas zu tun                                                                     '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub CancelButton_Click()
  
  Unload Me                                             ' Alles verwerfen, Dialog beenden

End Sub



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Fensterchen schliessen, ggf. neuen/geänderten Text übernehmen und "speichern?" Flag setzen                    '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub OKButton_Click()
  
  If (g_strCommentText <> Me.CommentsTextBox.Text) Then ' Wenn Text geändert, dann..
    g_strCommentText = Me.CommentsTextBox.Text          '..Text aus Textbox neu übernehmen und..
    g_bProfileChanged = True                            '..Abfrage "wollen speichern?"
  End If
  
  Unload Me                                             ' Dialog beenden

End Sub

