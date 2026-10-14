Attribute VB_Name = "NLGundPCKonstanten"
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / NLGundPCKonstanten                                                      '
'---------------------------------------------------------------------------------------------------------------'
' MODUL BESCHREIBUNG                  / Globale Konstanten, welche vom NLG und vom PC-Programm gebraucht werden.'
'                                     / Diese Konstanten werden 1:1 auch im C-File "NLG5_PC_Konst.h" definiert. '
'                                     / Die Code-Verdoppelung kommt daher, dass kein C- und VB-kompatibles..    '
'                                     /..Format für Definitions-Files gefunden wurde.                           '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' HISTORY   / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / 02.01.02  / Beat G.     / Von DatenKapsel Modul abgetrennt                                        '
' V1.08     / 26.11.02  / Beat G.     / bg010 neue Abschnittswechsel - Kriterien 61 und 62 eingebaut            '
' V2.0      / 23.05.03  / Beat G.     / bg019 neue Kriterien 11 und 21 eingebaut, + Vent.Bit für CP, ..         '
'                                     /.. + neu max 5 Bedingungen gleichzeitig (vorher: max. 3)                 '
' V3.0      / 17.09.04  / Beat G.     / bg022 neue Properties 40-46 eingebaut, CAN Baudrate und ID's wählbar    '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'
'
'
'
Option Explicit
'
'
'---------------- globale Konstanten für NLG- und PC-SW definieren -----------------------------------------------------------------

'---------------- "Eckwerte" der Kennlinie -----
Public Const ANZ_LADEABSCHNITTE As Integer = 7                    ' Anzahl Abschnitte der Lade - Kennlinie
Public Const ANZ_WECHSEL_BEDINGUNGEN = 5                          ' bg019 Maximale Anzahl Bedingungen für Wechsel zum nächsten/vorherigen Abschnitt
Public Const C_OR As Integer = 0                                  ' Verknüpfung 'oder'
Public Const C_AND As Integer = 1                                 ' Verknüpfung 'und'

'--------------- LED Zustände ---
Public Const LED_AUS    As Byte = 0                  ' LED ist ausgeschaltet
Public Const LED_BLINK  As Byte = 1                  ' LED blinkt
Public Const LED_EIN    As Byte = 2                  ' LED ist eingeschaltet

'--------------- Skalierungsfaktoren für Werte
Public Const SCALE_U_I          As Integer = 20     ' Ströme und Spannungen in der Regel mit Auflösung 0.05 (V, A)
Public Const SCALE_TEMP         As Integer = 2      ' Temperaturen in der Regel mit Auflösung 0.5 °C
Public Const SCALE_PWR          As Integer = 1      ' Leistung in der Regel mit Auflösung 1 Watt
Public Const SCALE_ENUM         As Integer = 1      ' Für Aufzählungen etc.
Public Const SCALE_SMALL_NEG    As Integer = -100   ' Auflösung 0.01, negativ (z.B. Koeff Gas-Spg)
Public Const SCALE_MINUTE       As Integer = 1      ' Zeiten in Minuten
Public Const SCALE_HOUR         As Integer = 60 * SCALE_MINUTE      ' Stunden werden auf Minuten abgebildet
Public Const SCALE_PERCENT      As Integer = 2      ' auf halbe Prozente genau darstellbar
Public Const SCALE_AH           As Integer = 20     ' Ampere-Stunden auf 0.05 Einheiten genau

'--------------- Werte für CAN Baudrate, 2-Byte unsigned Wert für Register BTR1 & BTR0
Public Const CAN_BAUDRATE_125k  As Long = &H7E03    ' BTR1=7Eh und BTR0=03 heisst 125 kBit/s
Public Const CAN_BAUDRATE_250k  As Long = &H7E01    ' BTR1=7Eh und BTR0=01 heisst 250 kBit/s
Public Const CAN_BAUDRATE_500k  As Long = &H7E80    ' BTR1=7Eh und BTR0=80 heisst 500 kBit/s (Default)
Public Const CAN_BAUDRATE_1M    As Long = &H2700    ' BTR1=27h und BTR0=00 heisst 1 MBit/s




'--------------- Kommunikations - Konstanten--------------------------------------------------------------------

' -------------- Kommunikations - Befehle
Public Const COM_STX  As Byte = 2                                 ' Start of Text (Daten Start) für Serielle Kommunikation
Public Const COM_ETX  As Byte = 3                                 ' End of Text (Daten Ende) für Serielle Kommunikation
Public Const COM_ACK  As Byte = 6                                 ' "Acknowledged" vom Gerät, "verstanden"
Public Const COM_NAK  As Byte = 21                                ' "Not Acknowledged" vom Gerät, "nicht verstanden"
  
Public Const COM_START_CHG  As Byte = 27                          ' Lade - Vorgang starten, "Esc"
Public Const COM_STOP_CHG   As Byte = 22                          ' Lade - Vorgang abbrechen, "Syn"
Public Const COM_LOG_OUT    As Byte = 4                           ' Kommunikation beenden, "EOT"

' -------------- Op_ID's, also Operationen auf die angesprochene Geräte - Eigenschaft
Public Const OP_SET   As Byte = 65                                ' Zeichen A (Ascii 65) für "Eigenschaft setzen"
Public Const OP_GET   As Byte = 66                                ' Zeichen B (Ascii 66) für "Eigenschaft abfragen"
Public Const OP_SEND  As Byte = 67                                ' Zeichen C (Ascii 67) für "abgefragte Eigenschaft zurückmelden"
Public Const OP_ERR   As Byte = 68                                ' Zeichen D (Ascii 68) für "kann Eigenschaft so nicht setzen"
Public Const OP_OK    As Byte = 69                                ' Zeichen E (Ascii 69) für "Ack oder Nak Meldung"

' --------------- Frame - Aufbau

' --- Positionen im Frame (von 0 (STX) weg gezählt!)
Public Const COM_POS_OP_ID As Byte = 1                            ' Position der Operation ID im Frame
Public Const COM_POS_PROP_HI As Byte = 2                          ' Position des Property_ID High Bytes
Public Const COM_POS_PROP_LO As Byte = 3                          ' Position des Property_ID Low Bytes
Public Const COM_POS_LENGTH As Byte = 4                           ' Position der Längenangabe im Frame
Public Const COM_POS_VAL_START As Byte = 5                        ' Start - Position des Werte-Strings im Frame

' --- weitere Grössen
Public Const COM_MAX_LEN As Byte = 25                             ' maximale Länge über Alles eines Kommunikations - Frames
Public Const COM_TRAIL_CHARS As Byte = 3                          ' Anzahl Bytes nach den Daten (CRC und End-Zeichen)
Public Const COM_OVERHEAD As Byte = COM_POS_VAL_START + COM_TRAIL_CHARS ' Anzahl Bytes, die zu den eigentlichen Daten dazukommen
Public Const MAX_PAR As Integer = 300                             ' Maximale Anzahl der Parameter (Eigenschaften mit Werten), bis jetzt bekannt: 286 (34 aus "Allg.Einst." + 252 aus 7 Abschn. "Generell/Vorw./Rückw.")
Public Const MAX_COND As Integer = 10                             ' Max Anzahl verschiedener Bedingungen (Texte und ID's), akt. Version: 10
'Public Const MAX_ACTIVE_COND As Integer = 3                       ' Max Anzahl gleichzeitig aktiver Bedingungen (Texte und ID's), akt. Version: 3


' --- Aufbau der Eigenschaften - Tabelle
Public Const MAX_RDONLY       As Long = &H13                      ' Grösstmögliche ReadOnly Prop_ID, wäre 19 dez
Public Const MAX_ALLG_PAR     As Long = &H63                      ' Grösstmögliche "Allgemeine Einstellung" Prop_ID, wäre 99 dez
Public Const OFFSET_ABSCHN    As Long = &H64                      ' hier beginnt der erste Lade-Abschnitt, 100 dez
Public Const LAENGE_ABSCHN    As Long = &H32                      ' soviele Einträge kann ein Abschnitt max haben, 50 dez
Public Const OFFSET_BEDING    As Long = 3                         ' eine Bedingung belegt 3 Einträge: Typ, Wert, Verknüpfung
Public Const DIRECT_CONTROL_CMDS As Long = 61440                  ' hier beginnen die Direct Control Commands   (Sch...VB versteht &HF000 nicht)
' Achtung: die Anzahl der tatsächlich benutzten Parameter/Properties NUM_USED_PAR ist eine Eigenschaft des PC-Programms und deshalb in der Datenkapsel definiert
  

' --------------- Properties, also die zu bearbeitenden Geräte - Eigenschaften (sehr viele!)
    
  ' ----------- Prop_ID's der Read-Only Gruppe
' ersetzt durch NLG-Typ, Serie-Nummer, Prod.Datum, FW-Version (numerisch)
'  Public Const DEVICE_NAME_0 As Long = &H1                        ' Hex 0001 für "Device Name 0"
'  Public Const DEVICE_NAME_1 As Long = &H2                        ' Hex 0002 für "Device Name 1"
'  Public Const DEVICE_NAME_2 As Long = &H3                        ' Hex 0003 für "Device Name 2"
'  Public Const DEVICE_NAME_3 As Long = &H4                        ' Hex 0004 für "Device Name 3"
  
  Public Const INF_NLG_TYP_51X      As Long = &H1                 ' Hex 0001 für X von "NLG-Typ 51X", 1 bedeutet also NLG 511
  Public Const INF_SER_NBR_MSB      As Long = &H2                 ' Hex 0002 für 2 höherwertige Byte der Serie-Nr
  Public Const INF_SER_NBR_LSB      As Long = &H3                 ' Hex 0003 für 2 niederwertige Byte der Serie-Nr
  Public Const INF_PROD_DD_MM_      As Long = &H4                 ' Hex 0004 für Produktionsdatum (Tag / Monat)
  Public Const INF_PROD_YYYY__      As Long = &H5                 ' Hex 0005 für Produktionsjahr (vierstellig)
  Public Const INF_FIRMWR_VERS      As Long = &H6                 ' Hex 0006 für Firmware Version (104 bedeutet V1.04)
  Public Const INF_FIRMWR_REV_      As Long = &H7                 ' Hex 0007 für Firmware Revisions-Index (03 bedeutet Version XX.YY.03)

  
  ' ----------- Prop_ID's der Allgemeinen Einstellungen
  Public Const BATT_NENN_SPANNG     As Long = &H14                ' Hex 0014 für "Nennspannung der Batterie"
  Public Const TEMP_KOEF_GAS_SP     As Long = &H15                ' Hex 0015 für "Temperatur - Koeffizient der Gasungsspannung"
  Public Const AKTIVE_TEMP_SENS     As Long = &H16                ' Hex 0016 für "Aktive Temperatur - Sensoren"
  Public Const BEZUGS_TEMP_SPNG     As Long = &H17                ' Hex 0017 für "Bezugstemperatur für Spannungen"
  Public Const MAX_NETZ_LEISTNG     As Long = &H18                ' Hex 0018 für "maximale Netzleistung"
  Public Const MAX_NETZ_STROM__     As Long = &H19                ' Hex 0019 für "maximaler Netzstrom"
  Public Const BETR_MASTR_BOOST     As Long = &H1A                ' Hex 001A für "Betriebsart (Standard/Master oder Booster)"
  Public Const EXT_STROM_P_MIN_     As Long = &H1B                ' Hex 001B für "Ext. Netzstrom für min. Booster - Leistung"
  Public Const EXT_STROM_P_MAX_     As Long = &H1C                ' Hex 001C für "Ext. Netzstrom für max. Booster - Leistung"
  Public Const TEMP_REDUKT_ART_     As Long = &H1D                ' Hex 001D für "Temperatur - Reduktions - Art"
  Public Const BATT_TEMP_DERATE     As Long = &H1E                ' Hex 001E für "Batterie - Temperatur -  Derating"
  Public Const TEMP_DERATE_NULL     As Long = &H1F                ' Hex 001F für "Bezugs - Temperatur für Derating auf Null"
  Public Const BAT_HYSTERES_EIN     As Long = &H20                ' Hex 0020 für "Einschaltpunkt für Batterie-Temp Hysterese"
  Public Const BAT_HYSTERES_AUS     As Long = &H21                ' Hex 0021 für "Ausschaltpunkt für Batterie-Temp Hysterese"
  Public Const BATT_TEMP_UEBER_     As Long = &H22                ' Hex 0022 für "Batterie - Temperatur über ... °C"
  Public Const BAT_SPANNG_UEBER     As Long = &H23                ' Hex 0023 für "Batterie - Spannung über ... V"
  Public Const NETZ_ENERGIE_UEB     As Long = &H24                ' Hex 0024 für "bezogene Netzenergie mehr als ... W"
  Public Const BAT_LAD_MEHR_ALS     As Long = &H25                ' Hex 0025 für "Batterie Ladung mehr als ... Ah"
  Public Const LAD_ZEIT_MER_ALS     As Long = &H26                ' Hex 0026 für "Ladezeit mehr als ... Std"

  
  ' ----------- Prop_ID's der Optionen (eigenes Fenster unter "Extras")
  Public Const VENTI_REQ_BIT_CP     As Long = &H27                ' bg019 Hex 0027 für "Control Pilot: Belüftung der Ladestation einschalten"
  Public Const CAN_BAUD_RATE___     As Long = &H28                ' bg022 Hex 0028 für "CAN Baud Rate"
  Public Const CAN_RX_ID_CTL___     As Long = &H29                ' bg022 Hex 0029 für "CAN Rx Msg ID NLG5_CTL"
  Public Const CAN_TX_ID_ST____     As Long = &H2A                ' bg022 Hex 002A für "CAN Tx Msg ID NLG5_ST"
  Public Const CAN_TX_ID_ACT_I_     As Long = &H2B                ' bg022 Hex 002B für "CAN Tx Msg ID NLG5_ACT_I"
  Public Const CAN_TX_ID_ACT_II     As Long = &H2C                ' bg022 Hex 002C für "CAN Tx Msg ID NLG5_ACT_II"
  Public Const CAN_TX_ID_TEMP__     As Long = &H2D                ' bg022 Hex 002D für "CAN Tx Msg ID NLG5_TEMP"
  Public Const CAN_TX_ID_ERR___     As Long = &H2E                ' bg022 Hex 002E für "CAN Tx Msg ID NLG5_ERR"
  
  
  ' ----------- Prop_ID's der Generellen Einstellungen (U-Phase / I-Phase, pro Abschnitt)
  Public Const ANZ_VORWAE_BED       As Long = &H64                ' Hex 0064 für "Anzahl Vorwärts - Bedingungen"
  Public Const ANZ_RUECKW_BED       As Long = &H65                ' Hex 0065 für "Anzahl Rückwärts - Bedingungen"
  Public Const MAX_I_PH_STROM       As Long = &H66                ' Hex 0066 für "I-Phase max Ladestrom"
  Public Const MAX_U_PH_SPANN       As Long = &H67                ' Hex 0067 für "U-Phase max Ladespannung"
  Public Const UEB_I_PH_STROM       As Long = &H68                ' Hex 0068 für "Überladung I-Phase in %"
  Public Const UEB_U_PH_SPANN       As Long = &H69                ' Hex 0069 für "Überladung U-Phase in %"
  Public Const I_STATUS_LEDS_       As Long = &H6A                ' Hex 006A für Status - Leds der I-Phase
  Public Const U_STATUS_LEDS_       As Long = &H6B                ' Hex 006B für Status - Leds der U-Phase
  Public Const ZU_AB1_EIN_AUS       As Long = &H6C                ' Hex 006C für "Zurück zu Abschnitt 1 bei Ein/Aus" Checkbox
  Public Const ZU_AB1_QUIT_1_       As Long = &H6D                ' Hex 006D für "Zurück zu Abschn. 1 bei Quittiersignal 1" Checkbox
  Public Const ZU_AB1_QUIT_2_       As Long = &H6E                ' Hex 006E für "Zurück zu Abschn. 1 bei Quittiersignal 2" Checkbox
  Public Const ZU_AB1_QUIT_12       As Long = &H6F                ' Hex 006F für "Zurück zu Abschn. 1 bei Quittiersignal 1+2" Checkbox
  
  ' ----------- Prop_ID's der Weiterschalt - Bedingungen (Spannung überschritten etc., pro Abschnitt)
  Public Const TYP_VORW_BED_1       As Long = &H70                ' "Typ der 1. Vorwärts - Bedingung"
  Public Const VAL_VORW_BED_1       As Long = &H71                ' "Wert der 1. Vorwärts - Bedingung"
  Public Const AND_OR_VORW_12       As Long = &H72                ' "Verknüpfung der 1. und 2. Vorwärts - Bedingung"
  Public Const TYP_VORW_BED_2       As Long = &H73                ' "Typ der 2. Vorwärts - Bedingung"
  Public Const VAL_VORW_BED_2       As Long = &H74                ' "Wert der 2. Vorwärts - Bedingung"
  Public Const AND_OR_VORW_23       As Long = &H75                ' "Verknüpfung der 2. und 3. Vorwärts - Bedingung"
  Public Const TYP_VORW_BED_3       As Long = &H76                ' "Typ der 3. Vorwärts - Bedingung"
  Public Const VAL_VORW_BED_3       As Long = &H77                ' "Wert der 3. Vorwärts - Bedingung"
  ' bg019 max 5 Kriterien
  Public Const AND_OR_VORW_34       As Long = &H78                ' "Verknüpfung der 3. und 4. Vorwärts - Bedingung"
  Public Const TYP_VORW_BED_4       As Long = &H79                ' "Typ der 4. Vorwärts - Bedingung"
  Public Const VAL_VORW_BED_4       As Long = &H7A                ' "Wert der 4. Vorwärts - Bedingung"
  Public Const AND_OR_VORW_45       As Long = &H7B                ' "Verknüpfung der 4. und 5. Vorwärts - Bedingung"
  Public Const TYP_VORW_BED_5       As Long = &H7C                ' "Typ der 5. Vorwärts - Bedingung"
  Public Const VAL_VORW_BED_5       As Long = &H7D                ' "Wert der 5. Vorwärts - Bedingung"

  Public Const MAX_COND_ID          As Integer = 999              ' Höchster zulässiger Identifier für Bedingungen
  Public Const COND_ID_SPANG_UEB    As Integer = 10               ' "Spannung ueberschritten [V]"
  Public Const COND_ID_SPG5_MIN_UNT As Integer = 11               ' "Spannungsanstieg in 5 Min. kleiner [V]"  bg019
  Public Const COND_ID_TEMP_UEB_    As Integer = 20               ' "Temperatur ueberschritten [°C]"
  Public Const COND_ID_TEMP_ANSTIEG As Integer = 21               ' "Temperaturanstieg überschritten [°C]"    bg019
  Public Const COND_ID_STROM_UNT    As Integer = 30               ' "Strom unterschritten [A]"
  Public Const COND_ID_LADZEIT_UEB  As Integer = 41               ' "Ladezeit in diesem Abschnitt überschritten [Min]"
  Public Const COND_ID_LADUNG_UEB   As Integer = 51               ' "Ladung überschritten [Ah] "
  Public Const COND_ID_REL_LAD_UEB  As Integer = 52               ' "Relative Ladung in diesem Abschnitt überschritten [%]"
  Public Const COND_ID_DIG_INPUT_3  As Integer = 61               ' "Digitaler Eingang DI3 Ein/Aus"   bg010
  Public Const COND_ID_DIG_INPUT_4  As Integer = 62               ' "Digitaler Eingang DI4 Ein/Aus"   bg010

  
  
  ' ----------- Prop_ID's der direkten NLG - Befehle (Direct Control Commands, DCC)
  Public Const DCC_CHARGE_START_STOP  As Long = DIRECT_CONTROL_CMDS       ' Hex F000 für "Batterie laden Start / Stop"
  Public Const DCC_EOT_LOG_OUT        As Long = DCC_CHARGE_START_STOP + 1 ' Hex F001 für "Logout / abmelden bei NLG"
