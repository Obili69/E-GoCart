Attribute VB_Name = "InitVonWerten"
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / InitVonWerten                                                           '
'---------------------------------------------------------------------------------------------------------------'
' MODUL BESCHREIBUNG                  / Globale Konstanten, globale Variablen, globale Typen, Stringtabellen..  '
'                                     /..usw. definieren. Dieses Modul stellt auch die Datenkapsel mit den..    '
'                                     /..entsprechenden Zugriffsfunktionen zur Verfügung                        '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' HISTORY   / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / Okt. '01  / Quang H.    / Rohfassung (Prinzipversuche)                                            '
' geändert  / Nov. '01  / Beat G.     / Konstanten für Protokoll RS232 und weitere eingefügt                    '
'                                     / Datensatz für Übertragung der Kennlinie                                 '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' MODUL - LISTE                       / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
' main                                / Bedingungen rückwärts/vorwärts in Array einfüllen                       '
' InitParArray                        / Array für Kennlinie mit Eigenschaften füllen                            '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'
'
'
'
'---------------- globale Konstanten definieren -----------------------------------------------------------------
Public Const ANZ_LADEABSCHNITTE As Integer = 7                    ' Anzahl Abschnitte der Lade - Kennlinie
Public Const ANZ_WECHSEL_BEDINGUNGEN As Integer = 5               ' Maximale Anzahl Bedingungen für Wechsel zum nächsten/vorherigen Abschnitt
Public Const AKTUALISIEREN_RUECKWAERTS As Integer = 4             ' Rückwärts Bedingungen neu anzeigen
Public Const AKTUALISIEREN_VORWAERTS As Integer = 5               ' Vorwärts Bedingungen neu anzeigen
Public Const AKTUALISIEREN_ABSCHNITT As Integer = 6               ' Ganzen Abschnitt neu anzeigen
Public Const NLG_MIN_KLASSE As Integer = 510                      ' Kleinste NLG Klasse
Public Const NLG_MAX_KLASSE As Integer = 516                      ' Grösste NLG Klasse
Public Const MAX_PAR As Integer = 30  '(rrr für debug: nur ein paar; richtig : 287)                        ' Maximale Anzahl der Parameter (Eigenschaften mit Werten)
Public Const MAX_RDONLY As Integer = 19                           ' Max ReadOnly Prop_ID
Public Const SHIFT_8_LEFT As Long = 256                           ' quasi nach links/rechts schieben, um 8 Binär-Stellen (Long, weil Bill hat Mühe mit TypeCasts)
Public Const MAX_BYTE As Byte = 255                               ' Grösstmöglicher Byte / Char - Wert
Public Const END_PAR As Byte = MAX_BYTE                           ' Zeichen für "Ende Datensatz", wenn Hi und Lo = END_PAR
Public Const NOT_PAR As Byte = 0                                  ' Zeichen für "Kein Parameter hier", wenn Hi und Lo = NOT_PAR


'--------------- Kommunikations - Konstanten--------------------------------------------------------------------

' -------------- Kommunikations - Befehle
Public Const COM_STX  As Byte = 2                                 ' Start of Text (Daten Start) für Serielle Kommunikation
Public Const COM_ETX  As Byte = 3                                 ' End of Text (Daten Ende) für Serielle Kommunikation
Public Const COM_ACK  As Byte = 6                                 ' "Acknowledged" vom Gerät, "verstanden"
Public Const COM_NAK  As Byte = 21                                ' "Not Acknowledged" vom Gerät, "nicht verstanden"
  
' -------------- Op_ID's, also Operationen auf die angesprochene Geräte - Eigenschaft
Public Const OP_SET   As Byte = 65                                ' Zeichen A (Ascii 65) für "Eigenschaft setzen"
Public Const OP_GET   As Byte = 66                                ' Zeichen B (Ascii 66) für "Eigenschaft abfragen"
Public Const OP_SEND  As Byte = 67                                ' Zeichen C (Ascii 67) für "abgefragte Eigenschaft zurückmelden"
Public Const OP_ERR   As Byte = 68                                ' Zeichen D (Ascii 68) für "kann Eigenschaft so nicht setzen"
Public Const OP_OK    As Byte = 69                                ' Zeichen E (Ascii 69) für "Ack oder Nak Meldung"

' --------------- Frame - Aufbau
                  ' --- Positionen im Frame (von 0 weg gezählt!)
Public Const COM_POS_OP_ID As Byte = 1                            ' Position der Operation ID im Frame
Public Const COM_POS_PROP_HI As Byte = 2                          ' Position des Property_ID High Bytes
Public Const COM_POS_PROP_LO As Byte = 3                          ' Position des Property_ID Low Bytes
Public Const COM_POS_LENGTH As Byte = 4                           ' Position de Längenangabe im Frame
Public Const COM_POS_VAL_START As Byte = 5                        ' Start - Position des Werte-Strings im Frame
                  ' --- weitere Grössen
Public Const COM_MAX_LEN As Byte = 25                             ' maximale Länge über Alles eines Kommunikations - Frames
Public Const COM_TRAIL_CHARS As Byte = 3                          ' Anzahl Bytes nach den Daten (CRC und End-Zeichen)
Public Const COM_OVERHEAD As Byte = COM_POS_VAL_START + COM_TRAIL_CHARS ' Anzahl Bytes, die zu den eigentlichen Daten dazukommen
  
  
' --------------- Properties, also die zu bearbeitenden Geräte - Eigenschaften (sehr viele!)
    
  ' ----------- Prop_ID's der Read-Only Gruppe
  Public Const DEVICE_NAME_0 As Long = &H1                        ' Hex 0000 für "Device Name 0"
  Public Const DEVICE_NAME_1 As Long = &H2                        ' Hex 0001 für "Device Name 1"
  Public Const DEVICE_NAME_2 As Long = &H3                        ' Hex 0002 für "Device Name 2"
  Public Const DEVICE_NAME_3 As Long = &H4                        ' Hex 0003 für "Device Name 3"
  
  ' ----------- Werte der Read-Only Gruppe                        "NLG 511 "
  Public Const VAL_DEVICE_NAME_0 As Long = &H4E4C                 ' Hex 4E 4C für "NL"
  Public Const VAL_DEVICE_NAME_1 As Long = &H4720                 ' Hex 47 20 für "G "
  Public Const VAL_DEVICE_NAME_2 As Long = &H3531                 ' Hex 35 31 für "51"
  Public Const VAL_DEVICE_NAME_3 As Long = &H3120                 ' Hex 31 20 für "1 "
  
  ' ----------- Prop_ID's der Allgemeinen Einstellungen
  Public Const BATT_NENN_SPANNG     As Long = &H14                ' Hex 0014 für "Nennspannung der Batterie"
  Public Const TEMP_KOEF_GAS_SP     As Long = &H15                ' Hex 0015 für "Temperatur - Koeffizient der Gasungsspannung"
  Public Const BEZUGS_TEMP_SPNG     As Long = &H16                ' Hex 0016 für "Bezugstemperatur für Spannungen"
  Public Const MAX_NETZ_LEISTNG     As Long = &H17                ' Hex 0017 für "maximale Netzleistung"
  Public Const MAX_NETZ_STROM__     As Long = &H18                ' Hex 0018 für "maximaler Netzstrom"
  Public Const TEMP_REDUKT_ART_     As Long = &H19                ' Hex 0019 für "Temperatur - Reduktions - Art"
  Public Const BATT_TEMP_DERATE     As Long = &H1A                ' Hex 001A für "Batterie - Temperatur -  Derating"
  Public Const TEMP_DERATE_NULL     As Long = &H1B                ' Hex 001B für "Bezugs - Temperatur für Derating auf Null"
  Public Const BAT_HYSTERES_EIN     As Long = &H1C                ' Hex 001C für "Einschaltpunkt für Batterie-Temp Hysterese"
  Public Const BAT_HYSTERES_AUS     As Long = &H1D                ' Hex 001D für "Ausschaltpunkt für Batterie-Temp Hysterese"
  Public Const BATT_TEMP_UEBER_     As Long = &H1E                ' Hex 001E für "Batterie - Temperatur über ... °C"
  Public Const BAT_SPANNG_UEBER     As Long = &H1F                ' Hex 001F für "Batterie - Spannung über ... V"
  Public Const NETZ_ENERGIE_UEB     As Long = &H20                ' Hex 0020 für "bezogene Netzenergie mehr als ... W"
  Public Const BAT_LAD_MEHR_ALS     As Long = &H21                ' Hex 0021 für "Batterie Ladung mehr als ... Ah"
  Public Const LAD_ZEIT_MER_ALS     As Long = &H22                ' Hex 0022 für "Ladezeit mehr als ... Std"

  ' ----------- Werte der Allgemeinen Einstellungen (rrr das muss dann natürlich aus Formular/Datei/NLG kommen !)
  Public Const VAL_BATT_NENN_SPANNG     As Long = 1500            ' Hex 05DC für "Nennspannung der Batterie = 75V"
  Public Const VAL_TEMP_KOEF_GAS_SP     As Long = 15              ' Hex 000F für "Temperatur - Koeffizient der Gasungsspannung"
  Public Const VAL_BEZUGS_TEMP_SPNG     As Long = 100             ' Hex 0064 für "Bezugstemperatur für Spannungen = 50°C"
  Public Const VAL_MAX_NETZ_LEISTNG     As Long = 3680            ' Hex 0E60 für "maximale Netzleistung 3.68 kW"
  Public Const VAL_MAX_NETZ_STROM__     As Long = 3200            ' Hex 0C80 für "maximaler Netzstrom = 16A"
  Public Const VAL_TEMP_REDUKT_ART_     As Long = &H0             ' Hex 0000 für "TemperaturReduktionsArt = Derating"
  Public Const VAL_BATT_TEMP_DERATE     As Long = 80              ' Hex 0050 für "TemperaturDerating = -4 Amp/°C"
  Public Const VAL_TEMP_DERATE_NULL     As Long = 100             ' Hex 0064 für "BezugsTemp. für Derating auf Null = 50 °C"
  Public Const VAL_BAT_HYSTERES_EIN     As Long = 88              ' Hex 0058 für "Einschaltpunkt BattTemp Hysterese = 44 °C"
  Public Const VAL_BAT_HYSTERES_AUS     As Long = 90              ' Hex 005A für "Ausschaltpunkt BattTemp Hysterese = 45 °C"
  Public Const VAL_BATT_TEMP_UEBER_     As Long = 110             ' Hex 006E für "Batterie - Temperatur über 55 °C"
  Public Const VAL_BAT_SPANNG_UEBER     As Long = 200             ' Hex 00C8 für "Batterie - Spannung über 100 V"
  Public Const VAL_NETZ_ENERGIE_UEB     As Long = 70              ' Hex 0046 für "bezogene Netzenergie mehr als 70 kWh"
  Public Const VAL_BAT_LAD_MEHR_ALS     As Long = 125             ' Hex 007C für "Batterie Ladung mehr als 125 Ah"
  Public Const VAL_LAD_ZEIT_MER_ALS     As Long = 24              ' Hex 0018 für "Ladezeit mehr als 24h"




'------------------------- globable Variablen ---------------------------------------------
Public com_port As Integer                                ' Kommunikations - Port
Public connected As Boolean                               ' Verbindung steht
Public comm_laenge As Integer                             ' wahrscheinlich verwaist
Public sprung As Integer                                  ' wahrscheinlich verwaist

Public rw(100, 4) As String                               ' Max 100 Rückwärts - Krit., 5 "Spalten": Texte, Min, Soll, Max, Einheit
Public vw(100, 4) As String                               ' Max 100 Vorwärts - Krit., 5 "Spalten": Texte, Min, Soll, Max, Einheit

'''Rueckwerts Abschnitt und wecksel Option(1-7,0-71)
Public rueckanz(7) As Integer                             ' Tatsächlich gewählte Anzahl Rückwärts - Bedingungen für diesen Abschnitt
'''Vorwerts Abschnitt und wecksel Option(1-7,0-71)
Public weiteranz(7) As Integer                            ' Tatsächlich gewählte Anzahl Vorwärts - Bedingungen für diesen Abschnitt
Public aktuelle_abschnitt As Integer                      ' Aktueller Abschnitt (in Bearbeitung)
Public INI_datei As String                                ' Dateiname

' neu bg: dies ist die eigentliche Datenkapsel
Public g_Par(MAX_PAR + 1) As tPropAndValue                ' Array mit Eigenschaften und zugehörigen Werten (-> "Parameter")
Public g_bMsgReceived As Boolean                          ' Poll - Flag für "Frame empfangen"





'------------------------- globale Typen -------------------------------------------------

'''' NLG Allgemeine  Einstellung
    Public Type nlgganz
        nennspannung    As Integer
        tempkoef        As Single
        bezugtemp       As Single
        maxnetl         As Integer
        maxneti         As Integer
        dh              As Boolean      '''0=Derating 1=hysterese
        dh1             As Integer      '''temp Bat derating/einschaltpunkt hysterese
        dh2             As Integer      '''Derating suf null/Ausschaltpunkt hysterese
        tempaus         As Integer
        spannungaus     As Integer
        mehralsi        As Integer
        mehralsah       As Integer
        ladezeitaus     As Integer
    End Type
    
    
    
'''' Abschnitt generelle Werte des Ladeabschnitts
    Public Type abschnittg
        'anzkritr        As Integer      '' Anzahl Kriterium zur vorherige Abschnitt
        'anzktirv        As Integer      '' Anzahl Kriterium zum nächsten Abschnitt
        imax            As Integer      ''  I maximum Ladestrom
        umax            As Integer      ''  U maximum Ladespannung
        uberladungi     As Integer      ''  Zählt für überladung in I(0/50/100%)
        uberladungu     As Integer      ''  Zählt für überladung in U(0/50/100%)
        zustandi        As String * 3   '' Zustand Anzeige I (3 Leds)
        zustandu        As String * 3   '' Zustand Anzeige U (3 Leds)
        zumab1          As String * 4       '' Zum Abschnitt 1 bei ein/ausschalten, Quittiersignal 1 oder 2  /  1 und 2
    End Type
    

'''' Kriterium für Ladeabschnitt
Public Type kriterium
        beschreibung    As Integer      ' Kriterium text (Beschreibung), eigentlich nummer von rw, "Bedingungs - ID"
        wert            As Variant      ' Wert von Bedingung  bg001 von "long" zu "variant" geändert, weil wir auch Dezimalstellen haben
        mitnexte        As Boolean      ' Verknüpfung mit nächster Bedingung (oder=0, und=1)
End Type

' Typ für Eigenschaft mit Wert ("Parameter"), daraus wird letztlich die ganze Kennlinie aufgebaut
Public Type tPropAndValue
  yPropHi        As Byte                 ' Eigenschaft High Byte
  yPropLo        As Byte                 ' Eigenschaft Low Byte
  lValue         As Long                 ' Wert des Parameters, der Bedingung usw.; ist eigentlich ein UNSIGNED INT,..
                                         '..den gibt's aber leider in VB nicht, also nehmen wir einen Long für 0h..FFFFh
  bWrite         As Boolean              ' darf der Wert überschrieben werden? (wenn False, dann ReadOnly)
End Type



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Bezeichner, Texte u.ä. für vorwärts und rückwärts Bedingungen der Kennlinie in Array füllen          '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Sub main()

Call InitParArray                       ' Array mit Parametern und Werten initialisieren'

' Beschreibung der 9 möglichen Rückwärts-Kriterien (Index x, 0)
' die Zehnerstelle von x bedeutet die Art des Kriteriums, die Einerstelle dessen Ausprägung
 rw(0, 0) = "Spannung unterschritten"
 rw(10, 0) = "Temperatur unterschritten"
 rw(20, 0) = "Ladestrom Überschritten"
 rw(30, 0) = "Abschnitt Timer abgelaufen"
 rw(40, 0) = "Abschnitt Ladungszähler abgelaufen"
 rw(50, 0) = "Dummy"
 rw(60, 0) = "Ah-Zähler überschritten"
 rw(70, 0) = "Eingang 1 aktiv"
 rw(71, 0) = "Eingang 2 Aktiv"
    
' Minima der Rückwärts-Kriterien (Index x, 1)
 rw(0, 1) = "-200"
 rw(10, 1) = "-10"
 rw(20, 1) = "-50"
 rw(30, 1) = "50"
 rw(40, 1) = "60"
 rw(50, 1) = "120"
 rw(60, 1) = "500"
 rw(70, 1) = "0"
 rw(71, 1) = "0"
    
' Soll der Rückwärts-Kriterien (Index x, 2)
' rrr sinnvolle Werte eintragen !
 rw(0, 2) = "0"
 rw(10, 2) = "1"
 rw(20, 2) = "2"
 rw(30, 2) = "3"
 rw(40, 2) = "4"
 rw(50, 2) = "5"
 rw(60, 2) = "6"
 rw(70, 2) = "7"
 rw(71, 2) = "8"

' Maxima der Rückwärts-Kriterien (Index x, 3)
 rw(0, 3) = "200"
 rw(10, 3) = "10"
 rw(20, 3) = "50"
 rw(30, 3) = "50"
 rw(40, 3) = "60"
 rw(50, 3) = "120"
 rw(60, 3) = "500"
 rw(70, 3) = "1"
 rw(71, 3) = "1"

' Einheit der Rückwärts-Kriterien (Index x, 4)
' rrr sinnvolle Werte eintragen !
 rw(0, 4) = "Volt"
 rw(10, 4) = "°C"
 rw(20, 4) = "Ampère"
 rw(30, 4) = "b"
 rw(40, 4) = "c"
 rw(50, 4) = "d"
 rw(60, 4) = "e"
 rw(70, 4) = "f"
 rw(71, 4) = "g"


' Beschreibung der vielen möglichen Vorwärts- Kriterien (Index x, 0)
' die Zehnerstelle von x bedeutet die Art des Kriteriums, die Einerstelle dessen Ausprägung
 vw(0, 0) = "Spannung überschritten"
 vw(1, 0) = "Abschnitt-Spannungsänderung unterschritten"
 vw(2, 0) = "Spannungsänderung/5min unterschritten"

 vw(10, 0) = "Temperatur überschritten"
 vw(11, 0) = "Gesamt-Temperaturänderung überschritten"
 vw(12, 0) = "Abschnitt-Temperaturänderung überschritten"
 vw(13, 0) = "Temperaturänderung unterschritten"

 vw(20, 0) = "Ladestrom unterschritten"
 vw(21, 0) = "Ladestrom-Abnahme/5min unterschritten"

 vw(30, 0) = "Gesamt Ladezeit überschritten"
 vw(31, 0) = "Absolute Abschnitts-Ladezeit überschritten"
 vw(32, 0) = "Rel. Anteil der bisherige Ladezeit überschritten"
 vw(33, 0) = "Abschnitt Timer abgelaufen"

 vw(40, 0) = "Gesamt-Ladung überschritten"
 vw(41, 0) = "Absolute Abschnitt-Ladung überschritten"
 vw(42, 0) = "REl. Anteil der bisherige Ladung überschritten"
 vw(43, 0) = "Abschnitt LAdungszähler abgelaufen"

 vw(50, 0) = "Zyklenzähler 1 abgelaufen"
 vw(51, 0) = "Zyklenzähler 2 abgelaufen"
 vw(52, 0) = "Kumulierter Gesamtladungszähler abgelaufen"
 vw(53, 0) = "Kumulierter Überladungszähler abgelaufen"

 vw(60, 0) = "Ah-Zählerstand unterschritten"
 vw(61, 0) = "Kumulierter Gesamt-Ah-Zähler abgelaufen"
 vw(62, 0) = "Kumulierter Überladungszähler abgelaufen"

 vw(70, 0) = "Eingang 1 aktiv"
 vw(71, 0) = "Eingang 2 aktiv"
 
 
' Minima der Vorwärts - Kriterien
' rrr sinnvolle Werte eintragen !
 vw(0, 1) = "-100"
 vw(1, 1) = "-100"
 vw(2, 1) = "-100"

 vw(10, 1) = "-100"
 vw(11, 1) = "-100"
 vw(12, 1) = "-100"
 vw(13, 1) = "-100"

 vw(20, 1) = "-100"
 vw(21, 1) = "-100"

 vw(30, 1) = "-100"
 vw(31, 1) = "-100"
 vw(32, 1) = "-100"
 vw(33, 1) = "-100"

 vw(40, 1) = "-100"
 vw(41, 1) = "-100"
 vw(42, 1) = "-100"
 vw(43, 1) = "-100"

 vw(50, 1) = "-100"
 vw(51, 1) = "-100"
 vw(52, 1) = "-100"
 vw(53, 1) = "-100"

 vw(60, 1) = "-100"
 vw(61, 1) = "-100"
 vw(62, 1) = "-100"

 vw(70, 1) = "-100"
 vw(71, 1) = "-100"
 
 
 ' Soll-Werte der Vorwärts-Kriterien
 ' rrr sinnvolle Werte eintragen !
 vw(0, 2) = "0"
 vw(1, 2) = "0"
 vw(2, 2) = "0"
 
 vw(10, 2) = "0"
 vw(11, 2) = "0"
 vw(12, 2) = "0"
 vw(13, 2) = "0"
 
 vw(20, 2) = "0"
 vw(21, 2) = "0"

 vw(30, 2) = "0"
 vw(31, 2) = "0"
 vw(32, 2) = "0"
 vw(33, 2) = "0"

 vw(40, 2) = "0"
 vw(41, 2) = "0"
 vw(42, 2) = "0"
 vw(43, 2) = "0"

 vw(50, 2) = "0"
 vw(51, 2) = "0"
 vw(52, 2) = "0"
 vw(53, 2) = "0"
 
 vw(60, 2) = "0"
 vw(61, 2) = "0"
 vw(62, 2) = "0"

 vw(70, 2) = "0"
 vw(71, 2) = "0"
 
 ' Maxima der Vorwärts-Kriterien
 ' rrr sinnvolle Werte eintragen !
 vw(0, 3) = "100"
 vw(1, 3) = "100"
 vw(2, 3) = "100"

 vw(10, 3) = "100"
 vw(11, 3) = "100"
 vw(12, 3) = "100"
 vw(13, 3) = "100"

 vw(20, 3) = "100"
 vw(21, 3) = "100"

 vw(30, 3) = "100"
 vw(31, 3) = "100"
 vw(32, 3) = "100"
 vw(33, 3) = "100"

 vw(40, 3) = "100"
 vw(41, 3) = "100"
 vw(42, 3) = "100"
 vw(43, 3) = "100"

 vw(50, 3) = "100"
 vw(51, 3) = "100"
 vw(52, 3) = "100"
 vw(53, 3) = "100"

 vw(60, 3) = "100"
 vw(61, 3) = "100"
 vw(62, 3) = "100"
 
 vw(70, 3) = "100"
 vw(71, 3) = "100"
 
 ''' Einheit von Werten
 
 vw(0, 4) = "Volt"
 vw(1, 4) = "Volt"
 vw(2, 4) = "Volt"

 vw(10, 4) = "C°"
 vw(11, 4) = "C°"
 vw(12, 4) = "C°"
 vw(13, 4) = "C°"

 vw(20, 4) = "Ampère"
 vw(21, 4) = "Ampère"

 vw(30, 4) = "Minuten"
 vw(31, 4) = "Minuten"
 vw(32, 4) = "Minuten"
 vw(33, 4) = "Minuten"

 ' rrr sinnvolle Werte eintragen !
 vw(40, 4) = "a"
 vw(41, 4) = "b"
 vw(42, 4) = "c"
 vw(43, 4) = "d"

 vw(50, 4) = "e"
 vw(51, 4) = "f"
 vw(52, 4) = "g"
 vw(53, 4) = "h"

 vw(60, 4) = "i"
 vw(61, 4) = "j"
 vw(62, 4) = "k"

 vw(70, 4) = "l"
 vw(71, 4) = "m"
 
 
 start_form.Show
 
End Sub

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Parameter Array der Kennlinie mit ID's und Werten füllen; hier wird die Zuordnung von Parameter-Nummern zu..  '
'..                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub InitParArray()
  
  Dim iCount As Integer
 
'---------- Zunächst mal alles mit Default Werten füllen -------------'
  
  For iCount = 0 To MAX_PAR + 1        ' Max. Anzahl Parameter, darf grösser als effektiv benutzte Anzahl sein (aber nicht kleiner!)
    g_Par(iCount).yPropHi = NOT_PAR
    g_Par(iCount).yPropLo = NOT_PAR
    g_Par(iCount).lValue = NOT_PAR
    g_Par(iCount).bWrite = True        ' Read/Write Parameter sind die Regel
  Next iCount
  
  ' Stelle 0 ist reserviert, nur lesbar
  g_Par(0).bWrite = False
  iCount = 1            ' nächster Parameter
  
  ' beginnend mit 1 werden nun alle Parameter fortlaufend in den Array eingefüllt;..
  '..das Array wird also "dynamisch erweitert", vorausgesetzt man stellt MAX_PAR genug gross ein

  
  
'---------- Die Read - Only Parameter --------------------------------'
  
  ' --- Name des Geräts ---
  
  ' Stelle 1
  g_Par(iCount).yPropLo = FnLoByteOfLong(DEVICE_NAME_0)
  g_Par(iCount).yPropHi = FnHiByteOfLong(DEVICE_NAME_0)
  g_Par(iCount).lValue = VAL_DEVICE_NAME_0
  g_Par(iCount).bWrite = False
  iCount = iCount + 1                 ' nächster Par

  ' Stelle 1
  g_Par(iCount).yPropLo = FnLoByteOfLong(DEVICE_NAME_1)
  g_Par(iCount).yPropHi = FnHiByteOfLong(DEVICE_NAME_1)
  g_Par(iCount).lValue = VAL_DEVICE_NAME_1
  g_Par(iCount).bWrite = False
  iCount = iCount + 1                 ' nächster Par

  ' Stelle 2
  g_Par(iCount).yPropLo = FnLoByteOfLong(DEVICE_NAME_2)
  g_Par(iCount).yPropHi = FnHiByteOfLong(DEVICE_NAME_2)
  g_Par(iCount).lValue = VAL_DEVICE_NAME_2
  g_Par(iCount).bWrite = False
  iCount = iCount + 1                 ' nächster Par

  ' Stelle 3
  g_Par(iCount).yPropLo = FnLoByteOfLong(DEVICE_NAME_3)
  g_Par(iCount).yPropHi = FnHiByteOfLong(DEVICE_NAME_3)
  g_Par(iCount).lValue = VAL_DEVICE_NAME_3
  g_Par(iCount).bWrite = False
  iCount = iCount + 1                 ' nächster Par




'---------- Die Read / Write - Parameter --------------------------------'

  ' --------Allgemeine Einstellungen
  
  ' Batterie Nennspannung
  g_Par(iCount).yPropLo = FnLoByteOfLong(BATT_NENN_SPANNG)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BATT_NENN_SPANNG)
  g_Par(iCount).lValue = VAL_BATT_NENN_SPANNG
  iCount = iCount + 1                 ' nächster Par

  ' Temp Koeffizient Gasungsspannung
  g_Par(iCount).yPropLo = FnLoByteOfLong(TEMP_KOEF_GAS_SP)
  g_Par(iCount).yPropHi = FnHiByteOfLong(TEMP_KOEF_GAS_SP)
  g_Par(iCount).lValue = VAL_TEMP_KOEF_GAS_SP
  iCount = iCount + 1                 ' nächster Par

  ' Bezugstemperatur der Gasungsspannung
  g_Par(iCount).yPropLo = FnLoByteOfLong(BEZUGS_TEMP_SPNG)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BEZUGS_TEMP_SPNG)
  g_Par(iCount).lValue = VAL_BEZUGS_TEMP_SPNG
  iCount = iCount + 1                 ' nächster Par

  ' Maximale Netzleistung
  g_Par(iCount).yPropLo = FnLoByteOfLong(MAX_NETZ_LEISTNG)
  g_Par(iCount).yPropHi = FnHiByteOfLong(MAX_NETZ_LEISTNG)
  g_Par(iCount).lValue = VAL_MAX_NETZ_LEISTNG
  iCount = iCount + 1                 ' nächster Par

  ' Maximaler Netzstrom
  g_Par(iCount).yPropLo = FnLoByteOfLong(MAX_NETZ_STROM__)
  g_Par(iCount).yPropHi = FnHiByteOfLong(MAX_NETZ_STROM__)
  g_Par(iCount).lValue = VAL_MAX_NETZ_STROM__
  iCount = iCount + 1                 ' nächster Par

  ' Temperatur Reduktionsart
  g_Par(iCount).yPropLo = FnLoByteOfLong(TEMP_REDUKT_ART_)
  g_Par(iCount).yPropHi = FnHiByteOfLong(TEMP_REDUKT_ART_)
  g_Par(iCount).lValue = VAL_TEMP_REDUKT_ART_
  iCount = iCount + 1                 ' nächster Par

  ' Batterie Temperatur Derating
  g_Par(iCount).yPropLo = FnLoByteOfLong(BATT_TEMP_DERATE)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BATT_TEMP_DERATE)
  g_Par(iCount).lValue = VAL_BATT_TEMP_DERATE
  iCount = iCount + 1                 ' nächster Par

  ' Bezugstemperatur für Derating auf Null
  g_Par(iCount).yPropLo = FnLoByteOfLong(TEMP_DERATE_NULL)
  g_Par(iCount).yPropHi = FnHiByteOfLong(TEMP_DERATE_NULL)
  g_Par(iCount).lValue = VAL_TEMP_DERATE_NULL
  iCount = iCount + 1                 ' nächster Par

  ' Einschaltpunkt für Batterie-Temp Hysterese
  g_Par(iCount).yPropLo = FnLoByteOfLong(BAT_HYSTERES_EIN)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BAT_HYSTERES_EIN)
  g_Par(iCount).lValue = VAL_BAT_HYSTERES_EIN
  iCount = iCount + 1                 ' nächster Par

  ' Ausschaltpunkt für Batterie-Temp Hysterese
  g_Par(iCount).yPropLo = FnLoByteOfLong(BAT_HYSTERES_AUS)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BAT_HYSTERES_AUS)
  g_Par(iCount).lValue = VAL_BAT_HYSTERES_AUS
  iCount = iCount + 1                 ' nächster Par

  ' Batterie - Temperatur über ... °C
  g_Par(iCount).yPropLo = FnLoByteOfLong(BATT_TEMP_UEBER_)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BATT_TEMP_UEBER_)
  g_Par(iCount).lValue = VAL_BATT_TEMP_UEBER_
  iCount = iCount + 1                 ' nächster Par

  ' Batterie - Spannung über ... V
  g_Par(iCount).yPropLo = FnLoByteOfLong(BAT_SPANNG_UEBER)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BAT_SPANNG_UEBER)
  g_Par(iCount).lValue = VAL_BAT_SPANNG_UEBER
  iCount = iCount + 1                 ' nächster Par

  ' bezogene Netzenergie mehr als ... W
  g_Par(iCount).yPropLo = FnLoByteOfLong(NETZ_ENERGIE_UEB)
  g_Par(iCount).yPropHi = FnHiByteOfLong(NETZ_ENERGIE_UEB)
  g_Par(iCount).lValue = VAL_NETZ_ENERGIE_UEB
  iCount = iCount + 1                 ' nächster Par

  ' Batterie Ladung mehr als ... Ah
  g_Par(iCount).yPropLo = FnLoByteOfLong(BAT_LAD_MEHR_ALS)
  g_Par(iCount).yPropHi = FnHiByteOfLong(BAT_LAD_MEHR_ALS)
  g_Par(iCount).lValue = VAL_BAT_LAD_MEHR_ALS
  iCount = iCount + 1                 ' nächster Par

  ' Ladezeit mehr als ... Std
  g_Par(iCount).yPropLo = FnLoByteOfLong(LAD_ZEIT_MER_ALS)
  g_Par(iCount).yPropHi = FnHiByteOfLong(LAD_ZEIT_MER_ALS)
  g_Par(iCount).lValue = VAL_LAD_ZEIT_MER_ALS
  iCount = iCount + 1                 ' nächster Par


'----------- Ende des Datensatzes -------------------------------------'
  g_Par(iCount).yPropHi = END_PAR
  g_Par(iCount).yPropLo = END_PAR
  g_Par(iCount).bWrite = False

End Sub





'' wahrscheinlich verwaiste Funktion
'Function max_karacter() As Integer
'    Dim k, l As Integer
'    l = 0
'    For i = 0 To 8
'        k = Len(rw(i, 0))
'        l = IIf(k > l, k, l)
'    Next i
'
'    For i = 0 To 100
'        k = Len(vw(i, 0))
'        l = IIf(k > l, k, l)
'    Next i
'    max_karacter = l
'End Function
