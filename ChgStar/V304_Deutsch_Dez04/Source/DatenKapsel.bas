Attribute VB_Name = "DatenKapselUndKonstanten"
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / DatenKapsel                                                             '
'---------------------------------------------------------------------------------------------------------------'
' MODUL BESCHREIBUNG                  / Dieses Modul stellt die Datenkapsel der Kennlinie mit den..             '
'                                     /..entsprechenden Zugriffsfunktionen zur Verfügung.                       '
'                                     /                                                                         '
'                                     / Auch globale Konstanten, globale Variablen, globale Typen,..            '
'                                     /..Stringtabellen usw. sind hier definiert. Konstanten, die auch vom NLG..'
'                                     /..gebraucht werden, stehen aber im Modul "NLG_5_Und_PC_Konstanten.bas" . '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' HISTORY   / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / Dez. '01  / Beat G.     / Datenkapsel inkl. Konstanten, Zugriffsfunktionen, Initialisierung etc.  '
' V1.08     / 26.11.02  / Beat G.     / bg010 neue Abschnittswechsel - Kriterien 61 und 62 eingebaut            '
' V2.0      / April '03 / Beat G.     / mehrsprachige Version (Deutsch / Englisch)                              '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
' main                                / Initialisierungen durchführen                                           '
' InitKriterien                       / verfügbare Kriterien (Text, Wertebereich etc.) festlegen                '
' InitParArray                        / Array für Kennlinie mit Eigenschaften füllen                            '
' PutDataToCapsule                    / 1 Parameter in die Datenkapsel schreiben                                '
' GetDataFromCapsule                  / 1 Parameter aus der Datenkapsel lesen                                   '
' GetStatusOfPar                      / Readable/Writeable Status eines Parameters lesen                        '
' GetNextPar                          / Nächsten Parameter in der Kapsel lesen                                  '
' FindParPos                          / Position eines Parameters in der Kapsel finden                          '
' GetScaleOfPar                       / Skalierung eines Parameters aus der Kapsel lesen                        '
' PutScaleToPar                       / Skalierung eines Parameters in die Kapsel schreiben                     '
' GetTextOfProperty                   / Text, der zu einer bestimmten Property gehört, für Datei etc.           '
' InitCapsuleToDefault                / Datenkapsel mit Default Werten initialisieren, für "Datei | neu"        '
' GetTextOfCondID                     / Text holen, der zu einem Bedingungstyp gehört                           '
' GetScaleOfCondID                    / Skalierung holen, die zu einem Bedingungstyp gehört                     '
' SetCorrectScaleIfCondIDType         / Skalierung setzen, falls ein Bedingungstyp in Kapsel geschrieben wird   '
'                                                                                                               '
'---------------------------------------------------------------------------------------------------------------'
'
'
'
'
' Public und Private in diesem Modul: die Idee ist, die Kapsel im engeren Sinn (also Daten und interne Funktionen)..
'..völlig privat zu halten. Nur die Zugriffsfunktionen sowie die "öffentlichen" Daten (v.a. Konstanten) sollen..
'..als Public exportiert und allen Modulen zugänglich sein. Public Funktionen sind mit je zwei Kommentar-Zeichen ('')..
'..im Header umrandet, Private mit nur einem.
'
'Bsp:
'
'       '''''''''''''''''''''''''''''''''''''
'       '''''''''''''''''''''''''''''''''''''
'       '' Public Function                 ''
'       '''''''''''''''''''''''''''''''''''''
'       '''''''''''''''''''''''''''''''''''''
'
'
'       '''''''''''''''''''''''''''''''''''''
'       ' Private Function                  '
'       '''''''''''''''''''''''''''''''''''''
'
'
Option Explicit
'
'
'---------------- globale Konstanten definieren -----------------------------------------------------------------

Public Const SWITCH_DEBUG As Boolean = False    ' rrr ddd Achtung: dies ist der DEBUG-Schalter, für Release auf FALSE setzen !

' aktuell benötigte Firmware - Version, um mit dieser Version von ChargeStar klarzukommen
Public Const REQUIRED_FIRMWARE_VERSION        As Long = 300


'--- Texte / Strings / Meldungen etc., länderspezifisch durch LocaleOffset ---
Public Const LOCALE_DEUTSCH                   As Long = 0
Public Const LOCALE_ENGLISH                   As Long = 10000


'' Alle String Identifier als Index-Nümmerli zur String Table im Resource File ("Zeichenfolgetabelle")
' Grund-Angaben
Public Const STR_ID_LANGUAGE                  As Long = 1000
Public Const STR_ID_APP_TITLE                 As Long = 1001
Public Const STR_ID_ANY_FAULT                 As Long = 1002

' Menüs
Public Const STR_ID_MNU_PROFILE               As Long = 1010
Public Const STR_ID_MNU_PROF_NEW              As Long = 1011
Public Const STR_ID_MNU_PROF_LOAD_FILE        As Long = 1012
Public Const STR_ID_MNU_PROF_SAVE_FILE        As Long = 1013
Public Const STR_ID_MNU_PROF_SAVE_NEW_FILE    As Long = 1014
Public Const STR_ID_MNU_PROF_RECEIVE_NLG      As Long = 1015
Public Const STR_ID_MNU_PROF_SEND_NLG         As Long = 1016
Public Const STR_ID_MNU_PROF_EXIT_PROG        As Long = 1017
Public Const STR_ID_MNU_CONNECT               As Long = 1030
Public Const STR_ID_MNU_CONN_NLG              As Long = 1031
Public Const STR_ID_MNU_DISCONN_NLG           As Long = 1032
Public Const STR_ID_MNU_EXTRAS                As Long = 1050
Public Const STR_ID_MNU_EXTRA_CHG_INFO        As Long = 1051
Public Const STR_ID_MNU_EXTRA_PROPERTIES      As Long = 1052
Public Const STR_ID_MNU_EXTRA_OPTIONS         As Long = 1053
Public Const STR_ID_MNU_HELP                  As Long = 1070
Public Const STR_ID_MNU_HLP_CHGST_HLP         As Long = 1071
Public Const STR_ID_MNU_HLP_CONT              As Long = 1072
Public Const STR_ID_MNU_HLP_ABOUT             As Long = 1073

' Verbindungs-Anzeige
Public Const STR_ID_RTX_NOT_CONN              As Long = 1090
Public Const STR_ID_RTX_CONNECTED             As Long = 1091
Public Const STR_ID_RTX_UNIT                  As Long = 1092

' Allgemeine Einstellungen
Public Const STR_ID_GENERAL_SETTINGS          As Long = 1100
Public Const STR_ID_GEN_NOM_BAT_VOLT          As Long = 1101
Public Const STR_ID_GEN_TEMP_KOEF_GAS         As Long = 1102
Public Const STR_ID_GEN_REF_TEMP_VOLT         As Long = 1103
Public Const STR_ID_GEN_ACT_TEMP_SENS         As Long = 1104
Public Const STR_ID_GEN_MAX_MAINS_CURR        As Long = 1105
Public Const STR_ID_GEN_TEMP_RED_MODE         As Long = 1106
Public Const STR_ID_GEN_MODE_DERATING         As Long = 1107
Public Const STR_ID_GEN_MODE_HYSTERESIS       As Long = 1108
Public Const STR_ID_GEN_BAT_TEMP_DERAT        As Long = 1109
Public Const STR_ID_GEN_REF_DERAT_ZERO        As Long = 1110
Public Const STR_ID_GEN_TEMP_HYSTER_ON        As Long = 1111
Public Const STR_ID_GEN_TEMP_HYSTER_OFF       As Long = 1112

' Betriebsart
Public Const STR_ID_OPERATION_MODE            As Long = 1120
Public Const STR_ID_OP_NLG_OP_MODE            As Long = 1121
Public Const STR_ID_OP_AUTO                   As Long = 1122
Public Const STR_ID_OP_BOOSTER                As Long = 1123
Public Const STR_ID_OP_CAN                    As Long = 1124
Public Const STR_ID_OP_CURR_BOOST_OFF         As Long = 1125
Public Const STR_ID_OP_CURR_BOOST_FULL        As Long = 1126

' Sicherheits-Abschaltungen
Public Const STR_ID_SAFETY                    As Long = 1140
Public Const STR_ID_SAFETY_BAT_TEMP           As Long = 1141
Public Const STR_ID_SAFETY_BAT_VOLT           As Long = 1142
Public Const STR_ID_SAFETY_CHARGE_AH          As Long = 1143
Public Const STR_ID_SAFETY_TIMEOUT            As Long = 1144

' Lade - Profil
Public Const STR_ID_CHARGING_PROFILE          As Long = 1200
Public Const STR_ID_PRF_PREVIOUS              As Long = 1201
Public Const STR_ID_PRF_SECTION               As Long = 1202
Public Const STR_ID_PRF_NEXT                  As Long = 1203
Public Const STR_ID_PRF_MAX_CHG_CURR          As Long = 1204
Public Const STR_ID_PRF_MAX_CHG_VOLT          As Long = 1205
Public Const STR_ID_PRF_NEXT_SECT_IF          As Long = 1206
Public Const STR_ID_PRF_OR                    As Long = 1207
Public Const STR_ID_PRF_AND                   As Long = 1208

' Umschalt - Bedingungen für Kennlinie
Public Const STR_ID_COND_OFFSET               As Long = 1300  ' Offset in der String-Table, wo die Bedingungen anfangen
Public Const STR_ID_COND_VOLT                 As Long = 1310
Public Const STR_ID_COND_VOLT_IN_5MIN         As Long = 1311
Public Const STR_ID_COND_TEMP                 As Long = 1320
Public Const STR_ID_COND_TEMP_RISE            As Long = 1321
Public Const STR_ID_COND_CURRENT              As Long = 1330
Public Const STR_ID_COND_TIME_IN_SECT         As Long = 1341
Public Const STR_ID_COND_CHG_IN_SECT          As Long = 1351
Public Const STR_ID_COND_REL_CHG_IN_SECT      As Long = 1352
Public Const STR_ID_COND_DI3_ON_OFF           As Long = 1361
Public Const STR_ID_COND_DI4_ON_OFF           As Long = 1362

' About Fenster
Public Const STR_ID_ABOUT_BOX_TITLE           As Long = 2000
Public Const STR_ID_ABOUT_DESCRIPTION         As Long = 2001
Public Const STR_ID_ABOUT_COPYLEFT            As Long = 2002
Public Const STR_ID_ABOUT_COPYRIGHT_NI        As Long = 2003

' Geräte Info
Public Const STR_ID_CHARGER_INFO_TITLE        As Long = 2100
Public Const STR_ID_CHG_INFO_TYPE             As Long = 2101
Public Const STR_ID_CHG_INFO_SERIAL           As Long = 2102
Public Const STR_ID_CHG_INFO_PROD_DATE        As Long = 2103
Public Const STR_ID_CHG_INFO_FW_VERSION       As Long = 2104

' Sende - Empfangs - Fenster
Public Const STR_ID_RTXBOX_TITLE              As Long = 2200
Public Const STR_ID_RTXBOX_WAIT_MSG           As Long = 2201
Public Const STR_ID_RTXBOX_SEND               As Long = 2202
Public Const STR_ID_RTXBOX_RECEIVE            As Long = 2203

' Optionen
Public Const STR_ID_OPTIONS_TITLE             As Long = 2300
Public Const STR_ID_OPTIONS_CP_FRAME          As Long = 2310
Public Const STR_ID_OPT_VENT_REQUEST          As Long = 2311
Public Const STR_ID_OPT_CAN_FRAME             As Long = 2320
Public Const STR_ID_OPT_LBL_BAUD              As Long = 2321
Public Const STR_ID_OPT_DEFAULT               As Long = 2322
Public Const STR_ID_OPT_LBL_RX_MSG            As Long = 2323
Public Const STR_ID_OPT_LBL_TX_MSG            As Long = 2324
Public Const STR_ID_OPT_LBL_RXTX_ID           As Long = 2325

' Eigenschaften
Public Const STR_ID_PROPERTIES_TITLE          As Long = 2400
Public Const STR_ID_PROP_PRINT_ALL            As Long = 2410

' Spannungs Konvertier Fenster
Public Const STR_ID_U_CONV_TITLE              As Long = 2500
Public Const STR_ID_U_CONV_TEXT               As Long = 2510
Public Const STR_ID_U_CONV_CHANGE_ALL         As Long = 2520
Public Const STR_ID_U_CONV_CHANGE_NOM         As Long = 2521

' Fehlermeldungen
Public Const STR_ID_ERR_FILE_CANT_SAVE        As Long = 3001
Public Const STR_ID_ERR_FILE_CANT_LOAD        As Long = 3002
Public Const STR_ID_ERR_FILE_NOT_VALID        As Long = 3003
Public Const STR_ID_ERR_CANT_CONNECT          As Long = 3004
Public Const STR_ID_ERR_PROPERTY__            As Long = 3005
Public Const STR_ID_ERR__CANT_RECEIVE         As Long = 3006
Public Const STR_ID_ERR__CANT_SEND            As Long = 3007
Public Const STR_ID_ERR_CANT_SEND_RESTART     As Long = 3008
Public Const STR_ID_ERR_RESTART_ACTIV_PROF    As Long = 3009
Public Const STR_ID_ERR_SER_PORT_IN_USE       As Long = 3010

' Sicherheitsabfragen / Meldungen
Public Const STR_ID_TXT_SAVE_CURR_PROF        As Long = 3020
Public Const STR_ID_TXT_NO_CONN_NLG           As Long = 3021
Public Const STR_ID_TXT_OLD_FW_NLG            As Long = 3022
Public Const STR_ID_NO_MORE_COND_ALLOWED      As Long = 3030

Public Const STR_ID_ERR_SER_PORT_NOT_READY    As Long = 3050
Public Const STR_ID_ERR_UNKNOWN_ERR__         As Long = 3051
Public Const STR_ID_ERR__OCCURRED             As Long = 3052

Public Const STR_ID_USE_DIFFERENT_CAN_ID      As Long = 3053

' Sonstige Texte
Public Const STR_ID_TXT_PROF_PFL              As Long = 3100
Public Const STR_ID_TXT_ALL_FILES             As Long = 3101
Public Const STR_ID_TXT_SAVE_FILE             As Long = 3102
Public Const STR_ID_TXT_LOAD_FILE             As Long = 3103
Public Const STR_ID_TXT_CURR_PROF             As Long = 3105
Public Const STR_ID_TXT_NEW_PROF              As Long = 3106
Public Const STR_ID_TXT_CANCEL                As Long = 3107

' Tooltip - Hilfetexte
Public Const STR_ID_HLP_ADD_COND              As Long = 3200
Public Const STR_ID_HLP_REMOVE_COND           As Long = 3201

' verschiedene Konstanten
Public Const PROFILE_GET As Boolean = False                       ' Kennlinie / Datensatz holen
Public Const PROFILE_SET As Boolean = True                        ' Kennlinie / Datensatz hinüberschicken
Public Const FILE_LOAD As Boolean = False                         ' Datei laden
Public Const FILE_SAVE As Boolean = True                          ' Datei speichern
Public Const UPDATE_ALL As Long = 0                               ' alles neu anzeigen
Public Const UPDATE_SECTION As Long = 1                           ' Abschnitte neu anzeigen
Public Const UPDATE_TEMPSENS As Long = 2                          ' nur Temp-Controls neu anzeigen
Public Const NORMAL_HEIGHT As Long = 8850                         ' Höhe Fenster, Standard-Betrieb
Public Const BOOSTER_HEIGHT As Long = 4550                        ' Höhe Fenster, Booster-Betrieb
Public Const MAX_WIDTH As Long = 10900                            ' Max Breite Fenster
Public Const BLINK_INTERVALL As Long = 400                        ' Blink - "Halbwelle" in ms
Public Const BLINKING As String = "blink"                         ' Zustand "blinkend"
Public Const NOT_BLINKING As String = ""                          ' Zustand "nicht blinkend"
Public Const FILE_LOCALE_ID As Long = &H807                       ' Datei schreiben/lesen: "Deutschschweizer Einstellungen"
Public Const MIN_BOOSTER_RAMPE As Integer = 4                     ' min/max Netzstrom master muss mindestens soweit auseinander sein

'----------- Werte der Properties mit Aufzählungstypen
Public Const C_STANDARD As Integer = 0                            ' Betriebsart Standard/Auto/Master
Public Const C_BOOSTER As Integer = 1                             ' Betriebsart Booster
Public Const C_CAN As Integer = 2                                 ' Betriebsart CAN

Public Const C_DERATING As Integer = 0                            ' Reduktionsart Derating
Public Const C_HYSTERESE As Integer = 1                           ' Reduktionsart Hysterese
Public Const C_OFF As Integer = 0                                 ' Temp Sensor AUS
Public Const C_ON As Integer = 1                                  ' Temp Sensor EIN

'----------- Indizierung der CAN Baudrate (Zuordnung zu Registerwerten separat)
Public Const INDEX_CAN_BAUD_125K               As Integer = 3
Public Const INDEX_CAN_BAUD_250K               As Integer = 2
Public Const INDEX_CAN_BAUD_500K               As Integer = 1
Public Const INDEX_CAN_BAUD_1M                 As Integer = 0

'----------- CAN Defaults -------------
Public Const DEFAULT_INDEX_CAN_BAUD            As Integer = INDEX_CAN_BAUD_500K    ' CAN Default ist 500kbd/s
Public Const DEFAULT_RX_CAN_ID_NLG5_CTL        As Integer = &H618       ' CAN Default Rx Id "Control" 618hex
Public Const DEFAULT_TX_CAN_ID_NLG5_ST         As Integer = &H610       ' CAN Default Tx Id "Status" 610hex
Public Const DEFAULT_TX_CAN_ID_NLG5_ACT_I      As Integer = &H611       ' CAN Default Tx Id "Actual Values 1" 611hex
Public Const DEFAULT_TX_CAN_ID_NLG5_ACT_II     As Integer = &H612       ' CAN Default Tx Id "Actual Values 2" 612hex
Public Const DEFAULT_TX_CAN_ID_NLG5_TEMP       As Integer = &H613       ' CAN Default Tx Id "Temperatures" 613hex
Public Const DEFAULT_TX_CAN_ID_NLG5_ERR        As Integer = &H614       ' CAN Default Tx Id "Errors + Warnings" 614hex
Public Const MIN_RX_TX_CAN_ID                  As Integer = &H0         ' kleinste zulässige CAN Tx Rx ID
Public Const MAX_RX_TX_CAN_ID                  As Integer = &H7FF       ' grösste zulässige CAN Tx Rx ID (2047dec)



' --------------- Fehlerbehandlung Datei Handling / Transmit etc. --------------

Public Const NO_ERROR As Long = 0                                 ' Ok, kein Fehler o.dgl.
Public Const ERROR_FILE_FORMAT = 1001                             ' Datei fehlerhaft
Public Const ERROR_FILE_SAVE As Long = 31036                      ' Datei kann nicht gespeichert werden
Public Const ERROR_FILE_LOAD As Long = 31037                      ' Datei kann nicht geladen werden
Public Const USER_FILE_QUIT As Long = 32755                       ' Datei öffnen/speichern vom user abgebrochen

Public Const ERR_RETRY As Integer = 4                             ' soviel mal wird versucht, die gleiche Property anzusprechen
Public Const LOGIN_RETRY As Integer = ERR_RETRY                   ' beim Login sind ev. mehr Versuche zugelassen als im Normalbetrieb
Public Const COM_TIMEOUT As Single = 0.4                          ' soviele Sekunden wird auf Antwort gewartet
Public Const LOG_TIMEOUT As Single = 0.3                          ' soviele Sekunden wird beim Ausloggen gewartet
Public Const LOGIN_WAIT As Single = 0.02                          ' soviele Sekunden zwischen Zeichen warten beim login
Public Const BUF_EMPTY As Integer = 0                             ' Buffer leer
Public Const CONNECT_ERROR As Long = 1100                         ' Fehler beim Verbindungsaufbau
Public Const FRAME_RECEIVE_ERROR As Long = 1101                   ' Fehler beim Empfang eines Frames, Retries abgelaufen
Public Const FRAME_SEND_ERROR As Long = 1102                      ' Fehler beim Senden eines Frames, Retries abgelaufen
Public Const CHARGE_STOP_ERROR As Long = 1110                     ' Fehler beim Senden von "Ladung sperren für Download Kennlinie"
Public Const CHARGE_START_ERROR As Long = 1111                    ' Fehler beim Senden von "Ladung wieder zulassen"

Public Const ALL_CONDITIONS_USED As Long = 1120                   ' logischer Fehler in Kennlinie: zuviele Bedingungen

Public Const PROFILE_DATA_CHANGE As Long = 1130                   ' du wollen speichern?
Public Const NO_CONNECTION_TO_NLG As Long = 1131                  ' nix verbindung da
Public Const OLD_FIRMWARE As Long = 1132                          ' Firmware Version älter als ChargeStar erwartet, ev. inkompatibel

'----------- mögliche NLG's
Public Const NLG_MIN_KLASSE As Integer = 510                      ' Kleinste NLG Klasse
Public Const NLG_MAX_KLASSE As Integer = 536                      ' Grösste NLG Klasse (i.d.R. "Fehler, nicht existent")
Public Const MIN_NOM_BATT_VOLT As Integer = 12                    ' kleinere Nennspannungen lassen wir nicht zu. Punkt.

Public Const SHIFT_8_LEFT As Long = 256                           ' quasi nach links/rechts schieben, um 8 Binär-Stellen (Long, weil Bill hat Mühe mit TypeCasts)
Public Const MAX_BYTE As Byte = 255                               ' Grösstmöglicher Byte / Char - Wert
Public Const MAX_UINT As Long = 65535                             ' Grösstmöglicher Unsigned Int (2 Byte) Wert (8ung: &hFFFF bedeutet in VB -1 !)
Public Const MAX_DAY As Long = 31                                 ' Grösstmöglicher Wert für Tag
Public Const MAX_MONTH As Long = 12                               ' Grösstmöglicher Wert für Monat
Public Const MAX_YEAR As Long = 2099                              ' Grösstmöglicher Wert für Jahr

Public Const END_PAR As Byte = MAX_BYTE                           ' Zeichen für "Ende Datensatz", wenn Hi und Lo = END_PAR
Public Const NOT_PAR As Byte = 0                                  ' Zeichen für "Kein Parameter hier", wenn Hi und Lo = NOT_PAR
Public Const NOT_TXT As String = ""                               ' Leer - String
Public Const STATE_ERROR As Integer = -1                          ' Fehler - Code bei Funktionen, die einen Status zurückgeben
Public Const STATE_OK As Integer = 1                              ' OK - Code bei Funktionen, die einen Status zurückgeben
Public Const STATE_IDLE As Integer = 0                            ' Zustand "inaktiv" für State Machine
Public Const STATE_A As Integer = 2                               ' Zustand A für "State Machine"
Public Const BTRUE As Integer = 1                                 ' "Wahr" in allen üblichen Maschinen - Sprachen (aber nicht VB)
Public Const BFALSE As Integer = 0                                ' "Falsch" in allen üblichen Maschinen - Sprachen (aber nicht VB)
Public Const NUM_USED_PAR As Integer = 160                        ' aktuell benutzte Parameter, nur für Anzeige relevant, darf auch etwas zu klein sein
Public Const NUM_USED_RDONLY As Integer = 4                       ' aktuell benutzte ReadOnly Parameter, nur für Anzeige relevant
  
'----------- Rückmeldung bei Spannungs-Konvertierungs-Dialog
Public Const STATE_WAIT_FOR_REACTION As Integer = 0               ' warten, bis der Benutzer etwas entschieden hat
Public Const STATE_ADAPT_ALL As Integer = 1                       ' alles anpassen
Public Const STATE_CHANGE_NOM_ONLY As Integer = 2                 ' nur Nennspannung ändern
Public Const STATE_CANCEL As Integer = 3                          ' Aktion abbrechen

  
' defaultmässig Werte gesetzt für Properties, in endgültiger Version ggf. deaktivieren, bzw. aus Datei / NLG / ... laden !
    
  ' ----------- Werte der Read-Only Gruppe                        "NLG 511 "
  Public Const VAL_INF_NLG_TYP_51X      As Long = &H9             ' Hex 9 für X von "NLG51X", also 519
  Public Const VAL_INF_SER_NBR_MSB      As Long = &H0             ' MSB der Serie-Nr (Def: 23456 dez = 5BA0 hex)
  Public Const VAL_INF_SER_NBR_LSB      As Long = &H5BA0          ' LSB der Serie-Nr (Def: 23456 dez = 5BA0 hex)
  Public Const VAL_INF_PROD_DD_MM_      As Long = &H1701          ' Prod.datum (Tag/Mt), def 23.01. = 17 01 hex
  Public Const VAL_INF_PROD_YYYY__      As Long = &H7D2           ' Prod.jahr (4-stellig), def 2002 dez = 7D2 hex
  Public Const VAL_INF_FIRMWR_VERS      As Long = &H3E7           ' Firmware Version, def. 999 dez = 3E7 hex
  Public Const VAL_INF_FIRMWR_REV_      As Long = &H63            ' Firmware Version, def. 99 dez = 63 hex
  
  ' ----------- Texte der Read-Only Gruppe
  Public Const TXT_PAR_ROOT             As String = "Par_Root"
  Public Const TXT_INF_NLG_TYP_51X      As String = "NLG_Typ_Last_Digit"  ' X von "NLG51X"
  Public Const TXT_INF_SER_NBR_MSB      As String = "NLG_Ser_Nr_MSB"      ' höherwertiges Byte der Serie-Nr
  Public Const TXT_INF_SER_NBR_LSB      As String = "NLG_Ser_Nr_LSB"      ' niederwertiges Byte der Serie-Nr
  Public Const TXT_INF_PROD_DD_MM_      As String = "NLG_Prod_Dat"        ' Produktionsdatum (Tag / Monat)
  Public Const TXT_INF_PROD_YYYY__      As String = "NLG_Prod_Jahr"       ' Produktionsjahr (vierstellig)
  Public Const TXT_INF_FIRMWR_VERS      As String = "NLG_Firmware_Vers"   ' Firmware Version
  Public Const TXT_INF_FIRMWR_REV_      As String = "NLG_Firmware_Revision" ' Firmware Revision Nbr

  ' ----------- Werte der "Allgemeinen Einstellungen"
  Public Const VAL_BATT_NENN_SPANNG     As Long = 1440            ' Batteriespannung 72V
  Public Const VAL_TEMP_KOEF_GAS_SP     As Long = 30              ' TempKoeff -0.30%/°C
  Public Const VAL_AKTIVE_TEMP_SENS     As Long = 100             ' TempSensor 1 aktiv, 2 und 3 passiv
  Public Const VAL_BEZUGS_TEMP_SPNG     As Long = 50              ' Bezugstemp 25°C
  Public Const VAL_MAX_NETZ_LEISTNG     As Long = 3680            ' PNetz max 3.68 kW
  Public Const VAL_MAX_NETZ_STROM__     As Long = 320             ' INetz max 16A
  Public Const VAL_BETR_MASTR_BOOST     As Long = C_STANDARD      ' Betriebsart Master (entspricht Auto / Standard)
  Public Const VAL_EXT_STROM_P_MIN_     As Long = 120             ' IExt für min Booster-Leistung = 6A
  Public Const VAL_EXT_STROM_P_MAX_     As Long = 200             ' IExt für max Booster-Leistung = 10A
  Public Const VAL_TEMP_REDUKT_ART_     As Long = C_DERATING      ' TemperaturReduktionsArt = Derating
  Public Const VAL_BATT_TEMP_DERATE     As Long = 400             ' TemperaturDerating = -4 Amp/°C
  Public Const VAL_TEMP_DERATE_NULL     As Long = 100             ' BezugsTemp. für Derating auf Null = 50 °C
  Public Const VAL_BAT_HYSTERES_EIN     As Long = 88              ' Einschaltpunkt BattTemp Hysterese = 44 °C
  Public Const VAL_BAT_HYSTERES_AUS     As Long = 90              ' Ausschaltpunkt BattTemp Hysterese = 45 °C
  Public Const VAL_BATT_TEMP_UEBER_     As Long = 110             ' Batterie - Temperatur über 55 °C
  Public Const VAL_BAT_SPANNG_UEBER     As Long = 1800            ' Batterie - Spannung über 90 V
  Public Const VAL_NETZ_ENERGIE_UEB     As Long = 70              ' bezogene Netzenergie mehr als 70 kWh
  Public Const VAL_BAT_LAD_MEHR_ALS     As Long = 125             ' Batterie Ladung mehr als 125 Ah
  Public Const VAL_LAD_ZEIT_MER_ALS     As Long = 1440            ' Ladezeit mehr als 24h (24 x 60Min = 1440)
  Public Const VAL_VENTI_REQ_BIT_CP     As Long = BFALSE          ' Belüftungs-Anforderung an CP per Default AUS
  Public Const VAL_CAN_BAUD_RATE___     As Long = CAN_BAUDRATE_500k             ' Per Default 500kBaud xxx
  Public Const VAL_CAN_RX_ID_CTL___     As Long = DEFAULT_RX_CAN_ID_NLG5_CTL    ' Per Default 618hex
  Public Const VAL_CAN_TX_ID_ST____     As Long = DEFAULT_TX_CAN_ID_NLG5_ST     ' Per Default 610hex
  Public Const VAL_CAN_TX_ID_ACT_I_     As Long = DEFAULT_TX_CAN_ID_NLG5_ACT_I  ' Per Default 611hex
  Public Const VAL_CAN_TX_ID_ACT_II     As Long = DEFAULT_TX_CAN_ID_NLG5_ACT_II ' Per Default 612hex
  Public Const VAL_CAN_TX_ID_TEMP__     As Long = DEFAULT_TX_CAN_ID_NLG5_TEMP   ' Per Default 613hex
  Public Const VAL_CAN_TX_ID_ERR___     As Long = DEFAULT_TX_CAN_ID_NLG5_ERR    ' Per Default 614hex
  
    ' ------- Texte zu "Allgemeine Einstellungen"
  Public Const TXT_BATT_NENN_SPANNG     As String = "Batterie_Nennspannung"
  Public Const TXT_TEMP_KOEF_GAS_SP     As String = "Temp_Koeff_Gasungsspannung"
  Public Const TXT_AKTIVE_TEMP_SENS     As String = "Anz_Aktive_Temperatursensoren"
  Public Const TXT_BEZUGS_TEMP_SPNG     As String = "Bezugs_Temp_Spannungen"
  Public Const TXT_MAX_NETZ_LEISTNG     As String = "Max_Netz_Leistung"
  Public Const TXT_MAX_NETZ_STROM__     As String = "Max_Netz_Strom"
  Public Const TXT_BETR_MASTR_BOOST     As String = "NLG_Betriebsart"
  Public Const TXT_EXT_STROM_P_MIN_     As String = "Ext_I_Min_Booster"
  Public Const TXT_EXT_STROM_P_MAX_     As String = "Ext_I_Max_Booster"
  Public Const TXT_TEMP_REDUKT_ART_     As String = "Temp_Reduktions_Art"
  Public Const TXT_BATT_TEMP_DERATE     As String = "Batt_Temp_Derating"
  Public Const TXT_TEMP_DERATE_NULL     As String = "Temp_Derating_Null"
  Public Const TXT_BAT_HYSTERES_EIN     As String = "Batt_Hysterese_Ein"
  Public Const TXT_BAT_HYSTERES_AUS     As String = "Batt_Hysterese_Aus"
  Public Const TXT_BATT_TEMP_UEBER_     As String = "Batt_Temp_Ueber"
  Public Const TXT_BAT_SPANNG_UEBER     As String = "Batt_Spannung_Ueber"
  Public Const TXT_NETZ_ENERGIE_UEB     As String = "Netz_Energie_Ueber"
  Public Const TXT_BAT_LAD_MEHR_ALS     As String = "Batt_Ladung_Mehr_Als"
  Public Const TXT_LAD_ZEIT_MER_ALS     As String = "Lade_Zeit_Mehr_Als"
  Public Const TXT_VENTI_REQ_BIT_CP     As String = "CP_Ventilation_Bit"
  Public Const TXT_CAN_BAUD_RATE___     As String = "CAN_Baud_Rate"
  Public Const TXT_CAN_RX_ID_CTL___     As String = "CAN_Rx_ID_CTL"
  Public Const TXT_CAN_TX_ID_ST____     As String = "CAN_Tx_ID_ST"
  Public Const TXT_CAN_TX_ID_ACT_I_     As String = "CAN_Tx_ID_ACT_I"
  Public Const TXT_CAN_TX_ID_ACT_II     As String = "CAN_Tx_ID_ACT_II"
  Public Const TXT_CAN_TX_ID_TEMP__     As String = "CAN_Tx_ID_TEMP"
  Public Const TXT_CAN_TX_ID_ERR___     As String = "CAN_Tx_ID_ERR"

  ' ----------- Werte  "Generell zu diesem Abschnitt"
  Public Const VAL_MAX_LADE_STROM       As Long = 0               ' I Max 25 A
  Public Const VAL_MAX_LADE_SPANNUNG    As Long = 0               ' U Max 90 V
  Public Const VAL_I_UBERLAD            As Long = 50              ' I zählt 50% für Überladung
  Public Const VAL_U_UBERLAD            As Long = 50              ' U zählt 50% für Überladung
  Public Const VAL_I_STATUS_LED         As Long = 1               ' "001" LED - Muster für I-Phase (Aus - Aus - Blink)
  Public Const VAL_U_STATUS_LED         As Long = 2               ' "002" LED - Muster für U-Phase (Aus - Aus - Ein)
  Public Const VAL_CHECK_AEG            As Long = 1               ' "1" für Aus/Ein Checkbox
  Public Const VAL_CHECK_QUIT_1         As Long = 0               ' "0" für Quit1 Checkbox
  Public Const VAL_CHECK_QUIT_2         As Long = 0               ' "0" für Quit2 Checkbox
  Public Const VAL_CHECK_QUIT_12        As Long = 1               ' "1" für Quit1+2 Checkbox
  
  
  ' ------- Texte zu "Generell zu diesem Abschnitt"
  Public Const TXT_MAX_LADE_STROM       As String = "Max_Lade_Strom"
  Public Const TXT_MAX_LADE_SPANNUNG    As String = "Max_Lade_Spannung"
  Public Const TXT_I_UBERLAD            As String = "I_Ueberladung_Prozent"
  Public Const TXT_U_UBERLAD            As String = "U_Ueberladung_Prozent"
  Public Const TXT_I_STATUS_LED         As String = "I_Status_LED"
  Public Const TXT_U_STATUS_LED         As String = "U_Status_LED"
  Public Const TXT_CHECK_AEG            As String = "Zu_Abschn_1_Aus_Ein"
  Public Const TXT_CHECK_QUIT_1         As String = "Zu_Abschn_1_Quit_1"
  Public Const TXT_CHECK_QUIT_2         As String = "Zu_Abschn_1_Quit_2"
  Public Const TXT_CHECK_QUIT_12        As String = "Zu_Abschn_1_Quit_1_und_2"

  
  ' ------- Texte zu "Wechsel - Bedingungen"
' bg019 max 5 Kriterien
  Public Const TXT_ANZ_VORW_BED         As String = "Anzahl_Vorwaerts_Bedingungen"
  Public Const TXT_ANZ_RUECK_BED        As String = "Anzahl_Rueckwaerts_Bedingungen"
  Public Const TXT_TYP_VORW_BED_1       As String = "Typ_Vorwaerts_Bedingung_1"
  Public Const TXT_TYP_VORW_BED_2       As String = "Typ_Vorwaerts_Bedingung_2"
  Public Const TXT_TYP_VORW_BED_3       As String = "Typ_Vorwaerts_Bedingung_3"
  Public Const TXT_TYP_VORW_BED_4       As String = "Typ_Vorwaerts_Bedingung_4"
  Public Const TXT_TYP_VORW_BED_5       As String = "Typ_Vorwaerts_Bedingung_5"
  Public Const TXT_WERT_VORW_BED_1      As String = "Wert_Vorwaerts_Bedingung_1"
  Public Const TXT_WERT_VORW_BED_2      As String = "Wert_Vorwaerts_Bedingung_2"
  Public Const TXT_WERT_VORW_BED_3      As String = "Wert_Vorwaerts_Bedingung_3"
  Public Const TXT_WERT_VORW_BED_4      As String = "Wert_Vorwaerts_Bedingung_4"
  Public Const TXT_WERT_VORW_BED_5      As String = "Wert_Vorwaerts_Bedingung_5"
  Public Const TXT_VKN_VORW_BED_12      As String = "Verknuepfung_Vorwaerts_Bed_12"
  Public Const TXT_VKN_VORW_BED_23      As String = "Verknuepfung_Vorwaerts_Bed_23"
  Public Const TXT_VKN_VORW_BED_34      As String = "Verknuepfung_Vorwaerts_Bed_34"
  Public Const TXT_VKN_VORW_BED_45      As String = "Verknuepfung_Vorwaerts_Bed_45"
 


'------------------------- krass globable Variablen ;-) ---------------------------------------------


' globale Variable für benutzte Sprache, kann von irgendwoher gesetzt werden
Public g_lLocaleOffset As Long

' höchst globale Flags für "Interprozess - Kommunikation"
Public g_bMsgReceived As Boolean                          ' Poll - Flag für "Frame empfangen"
Public g_bMsgStopTransmitData As Boolean                  ' Flag für Abbruch der Datenübertragung
Public g_strEmpfString     As String                      ' empfangene(s) Zeichen


Public g_lMsgVoltConversionState As Long                  ' Was hat der Benutzer bei Batt-Nenn-Spannungs-Konversion ausgewählt?

' globale Variablen für übergeordnete Funktionen
Public g_bProfileChanged As Boolean                       ' Flag für "Kennlinie geändert", wegen Anfrage "speichern?" und so
Public g_bConnectionEstablished As Boolean                ' ein- oder ausgeloggt?

Public g_iAktuellerAbschnitt As Integer                   ' Aktueller Abschnitt (in Bearbeitung)

' sonschtige Globalitäten
Public g_strCommentText As String                         ' Kommentar-Text in der Kennllinie wird in diesem String gehalten,..
                                                          '..gehört NICHT zur Datenkapsel (ist so 'ne Art FileInfo)
Public g_strProfileName As String                         ' Name der aktuellen Kennlinien-Datei, nur für Info-Zwecke!


' Dies ist die eigentliche Datenkapsel für den Kennlinien - Array. Dieser ist vorerst so ausgelegt, dass er Parameter mit..
'..einer Datenbreite von 2 Bytes aufnimmt; die Daten werden so gespeichert, wie sie auch im Gerät erscheinen (d.h. 2 Byte,..
'..mit Skalierung U/I: *20, T: *2, P: *1)


'------------------------- modulglobale Variablen ----------------------------------------

Public stPar(MAX_PAR + 1) As tstParPropAndValue           ' Array mit Eigenschaften und zugehörigen Werten (-> "Parameter")
Public stCond(MAX_COND + 1) As tstCondChangeSect          ' Array mit Texten und ID's der möglichen Abschnittswechsel Bedingungen



'------------------------- globale Typen -------------------------------------------------


' Struct Typ für "Parameter" (Eigenschaft mit Wert), daraus wird letztlich die ganze Kennlinie aufgebaut
Public Type tstParPropAndValue
  yPropHi       As Byte                 ' Eigenschaft High Byte
  yPropLo       As Byte                 ' Eigenschaft Low Byte
  lValue        As Long                 ' Wert des Parameters, der Bedingung usw.; ist eigentlich ein UNSIGNED INT,..
                                        '..den gibt's aber leider in VB nicht, also nehmen wir einen Long für 0h..FFFFh
  bWrite        As Boolean              ' darf der Wert überschrieben werden? (wenn False, dann ReadOnly)
  iScale        As Integer              ' die Werte sind mit der gleichen Skalierung abgelegt, wie sie im NLG verwendet wird
  strText       As String               ' Text, der zur Eigenschaft gehört
End Type


' Struct Typ für "Bedingung", verstanden als Abschnittswechsel-Bedingung; hier geht's primär um die Definition der..
'..möglichen Bedingungen; diese sind NICHT an bestimmte Properties (Nummern) gebunden
Public Type tstCondChangeSect
  iCondID       As Integer              ' identifiziert den Typ der Bedingung, z.B. "10" (für Spannung überschritten)
  iScale        As Integer              ' Skalierung der Werte, wie NLG
  strCondTxt    As String               ' Text zur Bedingung, z.B. "Spannung überschritten"

End Type

  





'----------------------------------------------- Öffentliche Funktionen --------------------------------------------------------------'


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Ein Parameter mit seinen Daten in Par-Array schreiben                                                          ''
''                                                                                                                ''
'' Eingang: ID's und Wert eines einzelnen Parameters                                                              ''
'' Ausgang: -                                                                                                     ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Sub PutDataToCapsule(ByVal yHiProp As Byte, ByVal yLoProp As Byte, ByVal lTheValue As Long)

  Dim iPos As Integer                     ' Position des Parameters im Array
  Dim lTmp As Long
  
  iPos = FindParPos(yHiProp, yLoProp)
  
  If (iPos > STATE_ERROR) Then            ' gültige Position?
    lTmp = FnLimit(lTheValue, 0, MAX_UINT)   ' Wert des zu schreibenden Parameters auf Bereich testen
    If (lTmp <> lTheValue) And (SWITCH_DEBUG = True) Then
      MsgBox ("Überlauf bei PutDataToCapsule")
    End If
    stPar(iPos).lValue = lTmp             ' bereichsgetesteten Wert des Parameters in Array schreiben
    
    ' Zusatz - Aufgabe: bei den "Abschnittswechsel - Parametern" bestimmt der Typ der Bedingung die Skalierung seines..
    '..Werts, welche ja normalerweise eine unverrückbare Eigenschaft des Parameters ist. Um hier möglichst konsistent..
    '..zur ursprünglichen Philosophie zu bleiben, setzen wir beim Auftreten des Bedingungs-Typs die Skalierung des..
    '..nachfolgenden Parameters (also des Werts der Bedingung) auf den passenden Skalierungs-Wert.
    ' Bsp: Prop112 = 10 (also Spannung überschritten), dann wird hier automatisch der .iScale von Prop113 auf 20..
    '..(also SCALE_U_I) gesetzt.
    lTmp = SetCorrectScaleIfCondIDType(yHiProp, yLoProp, lTheValue)
    
    If (lTmp < SCALE_ENUM) Then
      If (SWITCH_DEBUG) = True Then
        MsgBox ("Fehler bei SetCorrectScaleIfCondIDType")
      End If
    End If
    
  Else                                    ' ungültige Position
    If (SWITCH_DEBUG) = True Then
      MsgBox ("Fehler bei PutDataToCapsule")
    End If
  End If
  
  ' stPar().bWrite ist eine übergeordnete Parameter - Eigenschaft ("kann dieser Parameter im Gerät geändert werden?")..
  '..und kann mit dieser Funktion nicht verändert werden
  
End Sub


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Ein Parameter mit seinen Daten vom Par-Array lesen                                                             ''
''                                                                                                                ''
'' Eingang: PropID eines einzelnen Parameters                                                                     ''
'' Ausgang: Parameter - Wert als Long, der einen unsigned Int (0h-FFFFh) enthält                                  ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function GetDataFromCapsule(ByVal yHiProp As Byte, ByVal yLoProp As Byte) As Long

  Dim iPos As Integer                         ' Position des Parameters im Array
  Dim lTmp As Long
  
  iPos = FindParPos(yHiProp, yLoProp)
  
  If (iPos > STATE_ERROR) Then                ' Fehler
    lTmp = FnLimit(stPar(iPos).lValue, 0, MAX_UINT)   ' Wert des Parameters aus Array lesen und auf Bereich testen
    If (lTmp <> stPar(iPos).lValue) And (SWITCH_DEBUG = True) Then
      MsgBox ("Überlauf bei GetDataFromCapsule")
    End If
    GetDataFromCapsule = lTmp                 ' bereichsgetesteten Wert zurückgeben
  Else
    If SWITCH_DEBUG = True Then
      MsgBox ("Fehler bei GetDataFromCapsule")
    End If
    GetDataFromCapsule = STATE_ERROR          ' Fehler rückmelden
  End If

  
  
  ' stPar().bWrite ist eine übergeordnete Parameter - Eigenschaft ("kann dieser Parameter im Gerät geändert werden?")..
  '..und kann mit dieser Funktion nicht verändert werden
  
End Function


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Den Status eines Parameters vom Par-Array lesen                                                                ''
''                                                                                                                ''
'' Eingang: PropID eines einzelnen Parameters                                                                     ''
'' Ausgang: Read-Only(False) oder Read/Write(True), sowie Hi und Lo Byte des nächsten Parameters                  ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function GetStatusOfPar(ByVal yHiProp As Byte, ByVal yLoProp As Byte) As Boolean

  Dim iPos As Integer                         ' Position des Parameters im Array
  
  iPos = FindParPos(yHiProp, yLoProp)
  
  If (iPos > STATE_ERROR) Then                ' Fehler
    GetStatusOfPar = stPar(iPos).bWrite       ' Status des Parameters aus Array lesen
  Else
    If SWITCH_DEBUG = True Then
      MsgBox ("Fehler bei GetStatusOfPar")
    End If
    GetStatusOfPar = STATE_ERROR              ' Fehler rückmelden
  End If
  
  
End Function



''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Die Skalierung (Faktor zwischen "realem Wert" und seiner Darstellung im Gerät) eines Parameters vom Par-Array..''
'' holen.                                                                                                         ''
''                                                                                                                ''
'' Eingang: PropID eines einzelnen Parameters                                                                     ''
'' Ausgang: Skalierung (z.B. U oder I: in der Regel um 20 gedehnt) Achtung: keine "normale" Fehlermeldung..       ''
''..möglich, weil Skalierung auch negativ sein darf; stattdessen Wert "0" (kann keine gültige Skalierung sein)    ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function GetScaleOfPar(ByVal yHiProp As Byte, ByVal yLoProp As Byte) As Integer

  Dim iPos As Integer                         ' Position des Parameters im Array
  
  iPos = FindParPos(yHiProp, yLoProp)
  
  If (iPos > STATE_ERROR) Then                ' kein Fehler
    GetScaleOfPar = stPar(iPos).iScale        ' Skalierung des Parameters aus Array lesen
  Else
    If SWITCH_DEBUG = True Then
      MsgBox ("Fehler bei GetScaleOfPar")
    End If
    GetScaleOfPar = 0                         ' bedeutet: Fehler
  End If

End Function



''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Die Skalierung (Faktor zwischen "realem Wert" und seiner Darstellung im Gerät) eines Parameters im Par-Array.. ''
''..ändern. Wird eigentlich nur für die Abschnittswechsel - Bedingungen gebraucht.                                ''
''                                                                                                                ''
'' Eingang: - PropID eines einzelnen Parameters                                                                   ''
''          - Skalierung (z.B. U oder I: in der Regel um 20 gedehnt)                                              ''
'' Ausgang: Fehlerstatus (negativ = Fehler)                                                                       ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function PutScaleToPar(ByVal yHiProp As Byte, ByVal yLoProp As Byte, ByVal iScale As Integer) As Integer

  Dim iPos As Integer                         ' Position des Parameters im Array
  
  iPos = FindParPos(yHiProp, yLoProp)
  
  If (iPos > STATE_ERROR) Then                ' kein Fehler
    stPar(iPos).iScale = iScale               ' Skalierung des Parameters in Array schreiben
    PutScaleToPar = 1
  Else
    If SWITCH_DEBUG = True Then
      MsgBox ("Fehler bei PutScaleToPar")
    End If
    PutScaleToPar = -1                        ' bedeutet: Fehler
  End If

End Function


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Den nächsten Parameter vom Array liefern (wird für "durchzählende" Routinen gebraucht)                         ''
''                                                                                                                ''
'' Eingang: PropID eines einzelnen Parameters                                                                     ''
'' Ausgang: True (es gibt einen nächsten Par) oder False (es gibt keinen nächsten, das war der letzte)            ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function GetNextPar(ByVal yHiProp As Byte, ByVal yLoProp As Byte, ByRef yRetHi As Byte, ByRef yRetLo As Byte) As Boolean

  Dim iPos As Integer                         ' Position des Parameters im Array
  
  iPos = FindParPos(yHiProp, yLoProp)         ' aktuellen Parameter im Array suchen
  
  If (iPos > STATE_ERROR) Then                ' Ja, aktueller Parameter existiert
    iPos = iPos + 1                           ' nächsten Par anvisieren
    ' Abchecken, ob der nächste Parameter hieb- und stichfest in Ordnung ist
    If (((stPar(iPos).yPropHi > NOT_PAR) Or (stPar(iPos).yPropLo > NOT_PAR)) And ((stPar(iPos).yPropHi < END_PAR) Or (stPar(iPos).yPropLo < END_PAR))) Then
      GetNextPar = True                       ' ok
      yRetHi = stPar(iPos).yPropHi            ' Prop ID's vom nächsten Parameter retournieren
      yRetLo = stPar(iPos).yPropLo
    Else
      GetNextPar = False                      ' schon der aktuelle ist ungültig
    End If
  Else
    GetNextPar = False                        ' der aktuelle war wohl der Letzte
  End If
    
  
    
End Function




''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Den Text retournieren, der zu einer bestimmten Property gehört. Dieser Text wird z.B. dazu verwendet, beim..   ''
''..Speichern der Kennlinie auf Datei den entsprechenden Eintrag in der Datei zu identifizieren.                  ''
''                                                                                                                ''
'' Eingang: PropID eines einzelnen Parameters                                                                     ''
'' Ausgang: Text der Property                                                                                     ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function GetTextOfProperty(ByVal yHiProp As Byte, ByVal yLoProp As Byte) As String

  Dim iPos As Integer                         ' Position des Parameters im Array
  
  iPos = FindParPos(yHiProp, yLoProp)
  
  If (iPos > STATE_ERROR) Then                ' Fehler
    GetTextOfProperty = stPar(iPos).strText   ' Text des Parameters aus Array lesen
  Else
    If SWITCH_DEBUG = True Then
      MsgBox ("Fehler bei GetTextOfProperty")
    End If
    GetTextOfProperty = ""                    ' bedeutet: Fehler, ungültiger Parameter usw.
  End If

End Function




''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Die Datenkapsel mit Default - Werten füllen, wie's für die "Datei | Neu" Anweisung nötig ist                   ''
''                                                                                                                ''
'' Eingang: nix                                                                                                   ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Sub InitCapsuleToDefault()
  
  Call InitParArray(False)                ' Kennlinien - Array mit Parametern und Werten initialisieren, ohne NLG-Typ etc.
  Call InitKriterien                      ' Array mit den Definitionen der Kriterien initialisieren

End Sub



''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Text holen, der zu einem Bedingungstyp gehört (Abschnittswechsel - Bedingungen)                                ''
''                                                                                                                ''
'' Eingang: ID der Bedingung                                                                                      ''
'' (optional): international Version (beliebige Sprache) Ja oder Nein; wenn Nein, dann aktuelle Sprache           ''
'' (optional): Identifier für Sprache (wenn "international Version")                                              ''
''                                                                                                                ''
'' Ausgang: Text der Bedingung                                                                                    ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function GetTextOfCondID(ByVal lCondID As Long, Optional lIntlVers As Long, Optional lLocID As Long) As String

Dim lCount As Long                            ' Zählvar
  
  GetTextOfCondID = NOT_TXT                   ' Default: leerer String
  
  lCount = 0
                                                    
  If (lIntlVers = BTRUE) Then   ' "international Version", wird gebraucht fürs drucken; Wert in der Datenkapsel hängt..
    '..von der akt. Ländereinstellung ab; dieser Text hier ist hingegen in beliebiger Sprache erhältlich
    GetTextOfCondID = LoadResString(STR_ID_COND_OFFSET + lCondID + lLocID)
  Else                                        ' "normale" Version, baut auf Daten in Kapsel auf
    While ((lCount < MAX_COND) And (stCond(lCount).iCondID < END_PAR))  ' alle möglichen Bedingungen durchkämmen
      If (stCond(lCount).iCondID = lCondID) Then      ' gefragte Bedingung gefunden?
        GetTextOfCondID = stCond(lCount).strCondTxt   ' zugehörigen Text liefern
      End If
      lCount = lCount + 1                             ' nächste Bedingung checken
    Wend
  End If

End Function



''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'' Skalierung holen, die zu einem Bedingungstyp gehört (Abschnittswechsel - Bedingungen)                          ''
''                                                                                                                ''
'' Eingang: ID der Bedingung                                                                                      ''
''                                                                                                                ''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function GetScaleOfCondID(ByVal iCondID As Integer) As String

Dim iCount As Integer                           ' Zählvar
  
  GetScaleOfCondID = SCALE_ENUM                 ' Default: Aufzählung, also 1
  
  iCount = 0
                                                ' alle möglichen Bedingungen durchkämmen
  While ((iCount < MAX_COND) And (stCond(iCount).iCondID < END_PAR))
    If (stCond(iCount).iCondID = iCondID) Then  ' gefragte Bedingung gefunden?
      GetScaleOfCondID = stCond(iCount).iScale  ' zugehörige Skalierung liefern
    End If
    iCount = iCount + 1                         ' nächste Bedingung checken
  Wend

End Function




'--------------------------------------------------- Private Funktionen --------------------------------------------------------------'


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Alle notwendigen Initialisierungen der Datenkapsel bei Applikationsstart ausführen                            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Sub Main()
                                          ' hmpf
  'g_lLocaleOffset = LOCALE_ENGLISH        ' rrr setze Sprache fix auf Englisch
  g_lLocaleOffset = LOCALE_DEUTSCH        ' rrr setze Sprache fix auf Deutsch
  Call InitParArray(True)                 ' Kennlinien - Array mit Parametern und Werten komplett initialisieren
  Call InitKriterien                      ' Array mit den Definitionen der Kriterien initialisieren

End Sub


'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Position eines Parameters im Array bestimmen                                                                  '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
' N.B.: diese Funktion hat nicht gerade einen eleganten Such-Algorithmus. Da aber die zu durchsuchenden Datenfelder..
'..relativ klein sind, ist sie trotzdem recht schnell. Mit künstlich vergrösserten Feldern wurde gemessen (PC: 166MHz):
' 10000 Abfragen auf einen Array à 1000 Elemente dauerte ca. 29 Sekunden, wobei der Array fast immer vollständig durchsucht..
'..werden musste, weil's die Parameter ID nicht gab (Worst Case). Also nimmt ein Element ca. 3 us in Anspruch. Bei unseren..
'.. Feldgrössen (max. ca. 300) dauert somit eine Abfrage max 0.9 ms, alle Parameter damit max 270ms, das ist ok.
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Function FindParPos(ByVal yHiProp As Byte, ByVal yLoProp As Byte) As Integer
  
  FindParPos = 0
  
  While ((FindParPos < MAX_PAR) And (Not ((stPar(FindParPos).yPropHi = END_PAR) And (stPar(FindParPos).yPropLo = END_PAR))))
    If ((stPar(FindParPos).yPropHi = yHiProp) And (stPar(FindParPos).yPropLo = yLoProp)) Then
      Exit Function                                   ' Par ID's stimmen überein, also aktuelle Position zurückgeben
    End If
    FindParPos = FindParPos + 1                       ' Position inkrementieren
  Wend
  
  FindParPos = STATE_ERROR                            ' Parameter nicht gefunden
  
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Skalierung setzen, falls ein Bedingungstyp in die Kapsel geschrieben wird. Die Funktion nimmt den "Typ" als.. '
'..Property entgegen und setzt für den "Wert" (nachfolgende Property) die Skalierung                            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


Private Function SetCorrectScaleIfCondIDType(ByVal yHiProp As Byte, ByVal yLoProp As Byte, ByVal lTheValue As Long) As Integer
  
  
  Dim iSegCount As Integer, iCondCount As Integer       ' Zähler für Abschnitt und Bedingung
  Dim iScale As Integer                                 ' Skalierung
  Dim lCheckProp As Long                                ' zu prüfende Property
  
  SetCorrectScaleIfCondIDType = SCALE_ENUM              ' per Default: Skalierung 1

  For iSegCount = 0 To (ANZ_LADEABSCHNITTE - 1)         ' alle Abschnitte durchsuchen
    For iCondCount = 0 To (ANZ_WECHSEL_BEDINGUNGEN - 1) ' alle möglichen Bedingungen durchsuchen
      lCheckProp = TYP_VORW_BED_1 + (iSegCount * LAENGE_ABSCHN) + (iCondCount * 3)    ' PropID berechnen
      
      If (FnLongOf2Bytes(yHiProp, yLoProp) = lCheckProp) Then       ' PropID vergleichen
        iScale = GetScaleOfCondID(FnLimit(lTheValue, 0, MAX_COND_ID))  ' Skalierung zu diesem Typ holen
        lCheckProp = lCheckProp + 1                     ' wenn Prop gefunden, setze Skalierung der nächsten Prop
        SetCorrectScaleIfCondIDType = PutScaleToPar(FnHiByteOfLong(lCheckProp), FnLoByteOfLong(lCheckProp), iScale)
      End If
    
    Next iCondCount                                     ' nächste Bedingung
  Next iSegCount                                        ' nächster Abschnitt

End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Bezeichner, Texte u.ä. für vorwärts und rückwärts Bedingungen der Kennlinie in Array füllen                   '
' Hier können die verfügbaren Kriterien definiert, erweitert und eingeschränkt werden                           '
' Korrekt lokalisierte Texte dazu werden direkt aus der String-Tabelle (RES-Datei) geladen                      '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub InitKriterien()

Dim iCount As Integer

  iCount = 0
  
  

'' Umschalt - Bedingungen für Kennlinie
'Public Const STR_ID_COND_VOLT                 As Long = 1300
'Public Const STR_ID_COND_TEMP                 As Long = 1301
'Public Const STR_ID_COND_CURRENT              As Long = 1302
'Public Const STR_ID_COND_TIME_IN_SECT         As Long = 1303
'Public Const STR_ID_COND_CHG_IN_SECT          As Long = 1304
'Public Const STR_ID_COND_REL_CHG_IN_SECT      As Long = 1305
'Public Const STR_ID_COND_DI3_ON_OFF           As Long = 1306
  
  
  ' Die Bedingung "Spannung überschritten"
  stCond(iCount).iCondID = COND_ID_SPANG_UEB
  stCond(iCount).iScale = SCALE_U_I
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_VOLT + g_lLocaleOffset)
  iCount = iCount + 1
  
  ' Die Bedingung "Spannungsänderung in 5 Minuten unterschritten"
  stCond(iCount).iCondID = COND_ID_SPG5_MIN_UNT
  stCond(iCount).iScale = SCALE_U_I
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_VOLT_IN_5MIN + g_lLocaleOffset)
  iCount = iCount + 1
  
  ' Die Bedingung "Temperatur überschritten"
  stCond(iCount).iCondID = COND_ID_TEMP_UEB_
  stCond(iCount).iScale = SCALE_TEMP
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_TEMP + g_lLocaleOffset)
  iCount = iCount + 1
  
  ' Die Bedingung "Temperaturänderung überschritten"
  stCond(iCount).iCondID = COND_ID_TEMP_ANSTIEG
  stCond(iCount).iScale = SCALE_TEMP
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_TEMP_RISE + g_lLocaleOffset)
  iCount = iCount + 1
  
  ' Die Bedingung "Ladestrom unterschritten"
  stCond(iCount).iCondID = COND_ID_STROM_UNT
  stCond(iCount).iScale = SCALE_U_I
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_CURRENT + g_lLocaleOffset)
  iCount = iCount + 1
  
  ' Die Bedingung "Abschnitts - Ladezeit überschritten"
  stCond(iCount).iCondID = COND_ID_LADZEIT_UEB
  stCond(iCount).iScale = SCALE_MINUTE
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_TIME_IN_SECT + g_lLocaleOffset)
  iCount = iCount + 1
  
  ' Die Bedingung "Abschnitts - Ladung überschritten"
  stCond(iCount).iCondID = COND_ID_LADUNG_UEB
  stCond(iCount).iScale = SCALE_AH
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_CHG_IN_SECT + g_lLocaleOffset)
  iCount = iCount + 1
  
  ' Die Bedingung "relative Abschnitts - Ladung überschritten"
  stCond(iCount).iCondID = COND_ID_REL_LAD_UEB
  stCond(iCount).iScale = SCALE_PERCENT
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_REL_CHG_IN_SECT + g_lLocaleOffset)
  iCount = iCount + 1

  ' Die Bedingung "Digitaler Eingang DI3 Ein/Aus"   bg010
  stCond(iCount).iCondID = COND_ID_DIG_INPUT_3
  stCond(iCount).iScale = SCALE_ENUM
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_DI3_ON_OFF + g_lLocaleOffset)
  iCount = iCount + 1

  ' Die Bedingung "Digitaler Eingang DI4 Ein/Aus"   bg010
  stCond(iCount).iCondID = COND_ID_DIG_INPUT_4
  stCond(iCount).iScale = SCALE_ENUM
  stCond(iCount).strCondTxt = LoadResString(STR_ID_COND_DI4_ON_OFF + g_lLocaleOffset)
  iCount = iCount + 1

  ' Die Bedingung "keine weiteren Bedingungen"
  stCond(iCount).iCondID = END_PAR
  stCond(iCount).iScale = SCALE_ENUM
  stCond(iCount).strCondTxt = NOT_TXT

  
  If (SWITCH_DEBUG = True) Then
    start_form.Show                               ' Möglichkeit, ins Kommunikations - Hilfsprogramm zu gelangen
  Else
    kennlinie.Show                                ' sonst normales Kennlinienprogramm
  End If
  
End Sub                                       ' Ende InitKriterien



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Parameter Array der Kennlinie mit PropID's und Werten füllen; die Position iCount im Array ist willkürlich,.. '
'..der Zugriff erfolgt rein über die ID's, d.h. die Positionen müssen nicht bekannt sein                        '
' Der verfügbare Datensatz (welche Properties) wird hier festgelegt; er muss mit dem NLG übereinstimmen.        '
' Neue Properties werden hier eingetragen; ebenso können nicht mehr notwendige Properties hier entfernt werden. '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'
' In dieser Funktion sind alle Properties, Skalierungen usw. einzutragen, die benutzt werden sollen


Private Sub InitParArray(bFull As Boolean)
  
  Dim iCount As Integer
  Dim iLoop As Integer
 

  '---------- Zunächst mal alles mit Default Werten füllen -------------'
    
  If (bFull = True) Then                  ' komplette Initialisierung, nur am Anfang
    
    For iCount = 0 To MAX_PAR + 1         ' Max. Anzahl Parameter, darf grösser als effektiv benutzte Anzahl sein (aber nicht kleiner!)
      stPar(iCount).yPropHi = NOT_PAR
      stPar(iCount).yPropLo = NOT_PAR
      stPar(iCount).lValue = NOT_PAR
      stPar(iCount).bWrite = True         ' Read/Write Parameter sind die Regel
      stPar(iCount).iScale = SCALE_U_I    ' sind in der Regel für Ströme und Spannungen gleich (NLG5: 20, d.h. 1 Volt entspricht 20 LSB)
    Next iCount
    
    ' Stelle 0 ist reserviert, nur lesbar; Scale auf 1 (bei irrtümlicher Division nützlich ;-)
    stPar(0).bWrite = False
    stPar(0).iScale = 1
    stPar(0).strText = TXT_PAR_ROOT       ' das Ding hat'n Namen: "Par_Root"
    
  End If
    
  iCount = 1            ' nächster Parameter
    
  
  
  ' beginnend mit 1 werden nun alle Parameter fortlaufend in den (noch ungültigen) Array eingefüllt;..
    '..das gültige Array wird also "dynamisch erweitert", vorausgesetzt man stellt MAX_PAR genug gross ein
  
    
    
  '---------- Die Read - Only Parameter --------------------------------'
    
    ' --- Name des Geräts ---
    ' komplette Initialisierung, nur am Anfang
    ' Stelle 1
  If (bFull = True) Then              ' NLG Typ, X von "51X"
    stPar(iCount).yPropLo = FnLoByteOfLong(INF_NLG_TYP_51X)
    stPar(iCount).yPropHi = FnHiByteOfLong(INF_NLG_TYP_51X)
    stPar(iCount).lValue = VAL_INF_NLG_TYP_51X
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_INF_NLG_TYP_51X
    stPar(iCount).bWrite = False
  End If
  iCount = iCount + 1                 ' nächster Par
  
    ' Stelle 2
  If (bFull = True) Then              ' Serie - Nummer MSB
    stPar(iCount).yPropLo = FnLoByteOfLong(INF_SER_NBR_MSB)
    stPar(iCount).yPropHi = FnHiByteOfLong(INF_SER_NBR_MSB)
    stPar(iCount).lValue = VAL_INF_SER_NBR_MSB
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_INF_SER_NBR_MSB
    stPar(iCount).bWrite = False
  End If
  iCount = iCount + 1                 ' nächster Par
  
    
    ' Stelle 3
  If (bFull = True) Then              ' Serie - Nummer LSB
    stPar(iCount).yPropLo = FnLoByteOfLong(INF_SER_NBR_LSB)
    stPar(iCount).yPropHi = FnHiByteOfLong(INF_SER_NBR_LSB)
    stPar(iCount).lValue = VAL_INF_SER_NBR_LSB
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_INF_SER_NBR_LSB
    stPar(iCount).bWrite = False
  End If
  iCount = iCount + 1                 ' nächster Par
  
    ' Stelle 4
  If (bFull = True) Then              ' Produktions - Datum
    stPar(iCount).yPropLo = FnLoByteOfLong(INF_PROD_DD_MM_)
    stPar(iCount).yPropHi = FnHiByteOfLong(INF_PROD_DD_MM_)
    stPar(iCount).lValue = VAL_INF_PROD_DD_MM_
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_INF_PROD_DD_MM_
    stPar(iCount).bWrite = False
  End If
  iCount = iCount + 1                 ' nächster Par
  
    ' Stelle 5
  If (bFull = True) Then              ' Produktions - Jahr
    stPar(iCount).yPropLo = FnLoByteOfLong(INF_PROD_YYYY__)
    stPar(iCount).yPropHi = FnHiByteOfLong(INF_PROD_YYYY__)
    stPar(iCount).lValue = VAL_INF_PROD_YYYY__
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_INF_PROD_YYYY__
    stPar(iCount).bWrite = False
  End If
  iCount = iCount + 1                 ' nächster Par
    
    ' Stelle 6
  If (bFull = True) Then              ' Firmware Version
    stPar(iCount).yPropLo = FnLoByteOfLong(INF_FIRMWR_VERS)
    stPar(iCount).yPropHi = FnHiByteOfLong(INF_FIRMWR_VERS)
    stPar(iCount).lValue = VAL_INF_FIRMWR_VERS
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_INF_FIRMWR_VERS
    stPar(iCount).bWrite = False
  End If
  iCount = iCount + 1                 ' nächster Par
    
    ' Stelle 7
  If (bFull = True) Then              ' Firmware Version
    stPar(iCount).yPropLo = FnLoByteOfLong(INF_FIRMWR_REV_)
    stPar(iCount).yPropHi = FnHiByteOfLong(INF_FIRMWR_REV_)
    stPar(iCount).lValue = VAL_INF_FIRMWR_REV_
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_INF_FIRMWR_REV_
    stPar(iCount).bWrite = False
  End If
  iCount = iCount + 1                 ' nächster Par

  ' Ende "komplett initialisieren"
  
  
  
'---------- Die Read / Write - Parameter --------------------------------'

  ' --------Allgemeine Einstellungen
  
  ' Batterie Nennspannung
  stPar(iCount).yPropLo = FnLoByteOfLong(BATT_NENN_SPANNG)
  stPar(iCount).yPropHi = FnHiByteOfLong(BATT_NENN_SPANNG)
  stPar(iCount).lValue = VAL_BATT_NENN_SPANNG
  stPar(iCount).strText = TXT_BATT_NENN_SPANNG
  iCount = iCount + 1                 ' nächster Par

  ' Temp Koeffizient Gasungsspannung
  stPar(iCount).yPropLo = FnLoByteOfLong(TEMP_KOEF_GAS_SP)
  stPar(iCount).yPropHi = FnHiByteOfLong(TEMP_KOEF_GAS_SP)
  stPar(iCount).lValue = VAL_TEMP_KOEF_GAS_SP
  stPar(iCount).iScale = SCALE_SMALL_NEG
  stPar(iCount).strText = TXT_TEMP_KOEF_GAS_SP
  iCount = iCount + 1                 ' nächster Par

  ' Aktive Temperatur - Sensoren
  stPar(iCount).yPropLo = FnLoByteOfLong(AKTIVE_TEMP_SENS)
  stPar(iCount).yPropHi = FnHiByteOfLong(AKTIVE_TEMP_SENS)
  stPar(iCount).lValue = VAL_AKTIVE_TEMP_SENS
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_AKTIVE_TEMP_SENS
  iCount = iCount + 1                 ' nächster Par

  ' Bezugstemperatur der Gasungsspannung
  stPar(iCount).yPropLo = FnLoByteOfLong(BEZUGS_TEMP_SPNG)
  stPar(iCount).yPropHi = FnHiByteOfLong(BEZUGS_TEMP_SPNG)
  stPar(iCount).lValue = VAL_BEZUGS_TEMP_SPNG
  stPar(iCount).iScale = SCALE_TEMP
  stPar(iCount).strText = TXT_BEZUGS_TEMP_SPNG
  iCount = iCount + 1                 ' nächster Par

' deaktiviert, weil in aktueller Version nicht gebraucht (redundant mit Netzstrom)
'  ' Maximale Netzleistung
'  stPar(iCount).yPropLo = FnLoByteOfLong(MAX_NETZ_LEISTNG)
'  stPar(iCount).yPropHi = FnHiByteOfLong(MAX_NETZ_LEISTNG)
'  stPar(iCount).lValue = VAL_MAX_NETZ_LEISTNG
'  stPar(iCount).iScale = SCALE_PWR
'  stPar(iCount).strText = TXT_MAX_NETZ_LEISTNG
'  iCount = iCount + 1                 ' nächster Par

  ' Maximaler Netzstrom
  stPar(iCount).yPropLo = FnLoByteOfLong(MAX_NETZ_STROM__)
  stPar(iCount).yPropHi = FnHiByteOfLong(MAX_NETZ_STROM__)
  stPar(iCount).lValue = VAL_MAX_NETZ_STROM__
  stPar(iCount).strText = TXT_MAX_NETZ_STROM__
  iCount = iCount + 1                 ' nächster Par

  ' NLG Betriebsart (Master/Booster etc.)
  stPar(iCount).yPropLo = FnLoByteOfLong(BETR_MASTR_BOOST)
  stPar(iCount).yPropHi = FnHiByteOfLong(BETR_MASTR_BOOST)
  stPar(iCount).lValue = VAL_BETR_MASTR_BOOST
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_BETR_MASTR_BOOST
  iCount = iCount + 1                 ' nächster Par

  ' Ext Strom für minimale Booster Leistung
  stPar(iCount).yPropLo = FnLoByteOfLong(EXT_STROM_P_MIN_)
  stPar(iCount).yPropHi = FnHiByteOfLong(EXT_STROM_P_MIN_)
  stPar(iCount).lValue = VAL_EXT_STROM_P_MIN_
  stPar(iCount).strText = TXT_EXT_STROM_P_MIN_
  iCount = iCount + 1                 ' nächster Par

  ' Ext Strom für maximale Booster Leistung
  stPar(iCount).yPropLo = FnLoByteOfLong(EXT_STROM_P_MAX_)
  stPar(iCount).yPropHi = FnHiByteOfLong(EXT_STROM_P_MAX_)
  stPar(iCount).lValue = VAL_EXT_STROM_P_MAX_
  stPar(iCount).strText = TXT_EXT_STROM_P_MAX_
  iCount = iCount + 1                 ' nächster Par

  ' Temperatur Reduktionsart
  stPar(iCount).yPropLo = FnLoByteOfLong(TEMP_REDUKT_ART_)
  stPar(iCount).yPropHi = FnHiByteOfLong(TEMP_REDUKT_ART_)
  stPar(iCount).lValue = VAL_TEMP_REDUKT_ART_
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_TEMP_REDUKT_ART_
  iCount = iCount + 1                 ' nächster Par

  ' Batterie Temperatur Derating
  stPar(iCount).yPropLo = FnLoByteOfLong(BATT_TEMP_DERATE)
  stPar(iCount).yPropHi = FnHiByteOfLong(BATT_TEMP_DERATE)
  stPar(iCount).lValue = VAL_BATT_TEMP_DERATE
  stPar(iCount).iScale = SCALE_SMALL_NEG
  stPar(iCount).strText = TXT_BATT_TEMP_DERATE
  iCount = iCount + 1                 ' nächster Par

  ' Bezugstemperatur für Derating auf Null
  stPar(iCount).yPropLo = FnLoByteOfLong(TEMP_DERATE_NULL)
  stPar(iCount).yPropHi = FnHiByteOfLong(TEMP_DERATE_NULL)
  stPar(iCount).lValue = VAL_TEMP_DERATE_NULL
  stPar(iCount).iScale = SCALE_TEMP
  stPar(iCount).strText = TXT_TEMP_DERATE_NULL
  iCount = iCount + 1                 ' nächster Par

  ' Einschaltpunkt für Batterie-Temp Hysterese
  stPar(iCount).yPropLo = FnLoByteOfLong(BAT_HYSTERES_EIN)
  stPar(iCount).yPropHi = FnHiByteOfLong(BAT_HYSTERES_EIN)
  stPar(iCount).lValue = VAL_BAT_HYSTERES_EIN
  stPar(iCount).iScale = SCALE_TEMP
  stPar(iCount).strText = TXT_BAT_HYSTERES_EIN
  iCount = iCount + 1                 ' nächster Par

  ' Ausschaltpunkt für Batterie-Temp Hysterese
  stPar(iCount).yPropLo = FnLoByteOfLong(BAT_HYSTERES_AUS)
  stPar(iCount).yPropHi = FnHiByteOfLong(BAT_HYSTERES_AUS)
  stPar(iCount).lValue = VAL_BAT_HYSTERES_AUS
  stPar(iCount).iScale = SCALE_TEMP
  stPar(iCount).strText = TXT_BAT_HYSTERES_AUS
  iCount = iCount + 1                 ' nächster Par

  ' Batterie - Temperatur über ... °C
  stPar(iCount).yPropLo = FnLoByteOfLong(BATT_TEMP_UEBER_)
  stPar(iCount).yPropHi = FnHiByteOfLong(BATT_TEMP_UEBER_)
  stPar(iCount).lValue = VAL_BATT_TEMP_UEBER_
  stPar(iCount).iScale = SCALE_TEMP
  stPar(iCount).strText = TXT_BATT_TEMP_UEBER_
  iCount = iCount + 1                 ' nächster Par

  ' Batterie - Spannung über ... V
  stPar(iCount).yPropLo = FnLoByteOfLong(BAT_SPANNG_UEBER)
  stPar(iCount).yPropHi = FnHiByteOfLong(BAT_SPANNG_UEBER)
  stPar(iCount).lValue = VAL_BAT_SPANNG_UEBER
  stPar(iCount).strText = TXT_BAT_SPANNG_UEBER
  iCount = iCount + 1                 ' nächster Par

' deaktiviert, weil in aktueller Version nicht gebraucht (redundant mit Batterie - Ladung)
'  ' bezogene Netzenergie mehr als ... kWh
'  stPar(iCount).yPropLo = FnLoByteOfLong(NETZ_ENERGIE_UEB)
'  stPar(iCount).yPropHi = FnHiByteOfLong(NETZ_ENERGIE_UEB)
'  stPar(iCount).lValue = VAL_NETZ_ENERGIE_UEB
'  stPar(iCount).iScale = SCALE_ENUM
'  stPar(iCount).strText = TXT_NETZ_ENERGIE_UEB
'  iCount = iCount + 1                 ' nächster Par

  ' Batterie Ladung mehr als ... Ah
  stPar(iCount).yPropLo = FnLoByteOfLong(BAT_LAD_MEHR_ALS)
  stPar(iCount).yPropHi = FnHiByteOfLong(BAT_LAD_MEHR_ALS)
  stPar(iCount).lValue = VAL_BAT_LAD_MEHR_ALS
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_BAT_LAD_MEHR_ALS
  iCount = iCount + 1                 ' nächster Par

  ' Ladezeit mehr als ... Std
  stPar(iCount).yPropLo = FnLoByteOfLong(LAD_ZEIT_MER_ALS)
  stPar(iCount).yPropHi = FnHiByteOfLong(LAD_ZEIT_MER_ALS)
  stPar(iCount).lValue = VAL_LAD_ZEIT_MER_ALS
  stPar(iCount).iScale = SCALE_HOUR
  stPar(iCount).strText = TXT_LAD_ZEIT_MER_ALS
  iCount = iCount + 1                 ' nächster Par

  ' Ventilation Request Bit für den Control Pilot
  stPar(iCount).yPropLo = FnLoByteOfLong(VENTI_REQ_BIT_CP)
  stPar(iCount).yPropHi = FnHiByteOfLong(VENTI_REQ_BIT_CP)
  stPar(iCount).lValue = VAL_VENTI_REQ_BIT_CP
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_VENTI_REQ_BIT_CP
  iCount = iCount + 1                 ' nächster Par
  
  ' ---- bg022 Erweiterungen für CAN Parametrierung --------- xxx
  ' CAN Baud Rate
  stPar(iCount).yPropLo = FnLoByteOfLong(CAN_BAUD_RATE___)
  stPar(iCount).yPropHi = FnHiByteOfLong(CAN_BAUD_RATE___)
  stPar(iCount).lValue = VAL_CAN_BAUD_RATE___
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_CAN_BAUD_RATE___
  iCount = iCount + 1                 ' nächster Par

  ' CAN Rx ID, Control Msg
  stPar(iCount).yPropLo = FnLoByteOfLong(CAN_RX_ID_CTL___)
  stPar(iCount).yPropHi = FnHiByteOfLong(CAN_RX_ID_CTL___)
  stPar(iCount).lValue = VAL_CAN_RX_ID_CTL___
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_CAN_RX_ID_CTL___
  iCount = iCount + 1                 ' nächster Par

  ' CAN Tx ID, Status Msg
  stPar(iCount).yPropLo = FnLoByteOfLong(CAN_TX_ID_ST____)
  stPar(iCount).yPropHi = FnHiByteOfLong(CAN_TX_ID_ST____)
  stPar(iCount).lValue = VAL_CAN_TX_ID_ST____
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_CAN_TX_ID_ST____
  iCount = iCount + 1                 ' nächster Par

  ' CAN Rx ID, Actual Values 1
  stPar(iCount).yPropLo = FnLoByteOfLong(CAN_TX_ID_ACT_I_)
  stPar(iCount).yPropHi = FnHiByteOfLong(CAN_TX_ID_ACT_I_)
  stPar(iCount).lValue = VAL_CAN_TX_ID_ACT_I_
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_CAN_TX_ID_ACT_I_
  iCount = iCount + 1                 ' nächster Par

  ' CAN Rx ID, Actual Values 2
  stPar(iCount).yPropLo = FnLoByteOfLong(CAN_TX_ID_ACT_II)
  stPar(iCount).yPropHi = FnHiByteOfLong(CAN_TX_ID_ACT_II)
  stPar(iCount).lValue = VAL_CAN_TX_ID_ACT_II
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_CAN_TX_ID_ACT_II
  iCount = iCount + 1                 ' nächster Par

  ' CAN Rx ID, Temperatures
  stPar(iCount).yPropLo = FnLoByteOfLong(CAN_TX_ID_TEMP__)
  stPar(iCount).yPropHi = FnHiByteOfLong(CAN_TX_ID_TEMP__)
  stPar(iCount).lValue = VAL_CAN_TX_ID_TEMP__
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_CAN_TX_ID_TEMP__
  iCount = iCount + 1                 ' nächster Par

  ' CAN Rx ID, Errors
  stPar(iCount).yPropLo = FnLoByteOfLong(CAN_TX_ID_ERR___)
  stPar(iCount).yPropHi = FnHiByteOfLong(CAN_TX_ID_ERR___)
  stPar(iCount).lValue = VAL_CAN_TX_ID_ERR___
  stPar(iCount).iScale = SCALE_ENUM
  stPar(iCount).strText = TXT_CAN_TX_ID_ERR___
  iCount = iCount + 1                 ' nächster Par
  ' ---- End bg022 Erweiterungen für CAN Parametrierung ---------
  

  
  ' ------------ "über Sieben Abschnitte musst Du gehn..." ----------
  
  For iLoop = 0 To (ANZ_LADEABSCHNITTE - 1)
  
    ' -------- Generelle Einstellungen -----------------
    
    ' Anzahl Vorwärts - Bedingungen für Abschnittswechsel
    stPar(iCount).yPropLo = FnLoByteOfLong(ANZ_VORWAE_BED + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(ANZ_VORWAE_BED + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = 0
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_ANZ_VORW_BED
    iCount = iCount + 1                 ' nächster Par
  
    ' Anzahl Rückwärts - Bedingungen für Abschnittswechsel
    stPar(iCount).yPropLo = FnLoByteOfLong(ANZ_RUECKW_BED + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(ANZ_RUECKW_BED + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = 0
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_ANZ_RUECK_BED
    iCount = iCount + 1                 ' nächster Par

    ' --------- U/I-Phase ---------------------------------------------

    ' Max.Ladestrom in A
    stPar(iCount).yPropLo = FnLoByteOfLong(MAX_I_PH_STROM + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(MAX_I_PH_STROM + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = VAL_MAX_LADE_STROM
    stPar(iCount).strText = TXT_MAX_LADE_STROM
    iCount = iCount + 1                 ' nächster Par

    ' Max. Ladespannung in V
    stPar(iCount).yPropLo = FnLoByteOfLong(MAX_U_PH_SPANN + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(MAX_U_PH_SPANN + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = VAL_MAX_LADE_SPANNUNG
    stPar(iCount).strText = TXT_MAX_LADE_SPANNUNG
    iCount = iCount + 1                 ' nächster Par

'    ' I Zählt für Überladung...%
'    stPar(iCount).yPropLo = FnLoByteOfLong(UEB_I_PH_STROM + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).yPropHi = FnHiByteOfLong(UEB_I_PH_STROM + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).lValue = VAL_I_UBERLAD
'    stPar(iCount).iScale = SCALE_ENUM
'    stPar(iCount).strText = TXT_I_UBERLAD
'    iCount = iCount + 1                 ' nächster Par
'
'    ' U Zählt für Überladung...%
'    stPar(iCount).yPropLo = FnLoByteOfLong(UEB_U_PH_SPANN + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).yPropHi = FnHiByteOfLong(UEB_U_PH_SPANN + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).lValue = VAL_U_UBERLAD
'    stPar(iCount).iScale = SCALE_ENUM
'    stPar(iCount).strText = TXT_U_UBERLAD
'    iCount = iCount + 1                 ' nächster Par
    
    
    ' ------------ Status Leds ---------------------------

    ' Status Leds I-Phase
    stPar(iCount).yPropLo = FnLoByteOfLong(I_STATUS_LEDS_ + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(I_STATUS_LEDS_ + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = VAL_I_STATUS_LED
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_I_STATUS_LED
    iCount = iCount + 1                 ' nächster Par

    ' Status Leds U-Phase
    stPar(iCount).yPropLo = FnLoByteOfLong(U_STATUS_LEDS_ + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(U_STATUS_LEDS_ + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = VAL_U_STATUS_LED
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_U_STATUS_LED
    iCount = iCount + 1                 ' nächster Par
    
    
    
    ' deaktiviert, weil in aktueller Version nicht gebraucht
'    ' ------------ Checkboxen "zurück zu Abschnitt 1"
'
'    ' Zu Abschnitt 1 bei Aus/Einschalten
'    stPar(iCount).yPropLo = FnLoByteOfLong(ZU_AB1_EIN_AUS + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).yPropHi = FnHiByteOfLong(ZU_AB1_EIN_AUS + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).lValue = VAL_CHECK_AEG
'    stPar(iCount).iScale = SCALE_ENUM
'    stPar(iCount).strText = TXT_CHECK_AEG
'    iCount = iCount + 1                 ' nächster Par
'
'    ' Zu Abschnitt 1 bei Quittiersignal 1
'    stPar(iCount).yPropLo = FnLoByteOfLong(ZU_AB1_QUIT_1_ + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).yPropHi = FnHiByteOfLong(ZU_AB1_QUIT_1_ + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).lValue = VAL_CHECK_QUIT_1
'    stPar(iCount).iScale = SCALE_ENUM
'    stPar(iCount).strText = TXT_CHECK_QUIT_1
'    iCount = iCount + 1                 ' nächster Par
'
'
'    ' Zu Abschnitt 1 bei Quittiersignal 2
'    stPar(iCount).yPropLo = FnLoByteOfLong(ZU_AB1_QUIT_2_ + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).yPropHi = FnHiByteOfLong(ZU_AB1_QUIT_2_ + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).lValue = VAL_CHECK_QUIT_2
'    stPar(iCount).iScale = SCALE_ENUM
'    stPar(iCount).strText = TXT_CHECK_QUIT_2
'    iCount = iCount + 1                 ' nächster Par
'
'    ' Zu Abschnitt 1 bei Quittiersignal 1+2
'    stPar(iCount).yPropLo = FnLoByteOfLong(ZU_AB1_QUIT_12 + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).yPropHi = FnHiByteOfLong(ZU_AB1_QUIT_12 + (iLoop * LAENGE_ABSCHN))
'    stPar(iCount).lValue = VAL_CHECK_QUIT_12
'    stPar(iCount).iScale = SCALE_ENUM
'    stPar(iCount).strText = TXT_CHECK_QUIT_12
'    iCount = iCount + 1                 ' nächster Par
    
    
    ' ----------------- Wechsel - Bedingungen -------------------------
   
    
    ' Typ der 1.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(TYP_VORW_BED_1 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(TYP_VORW_BED_1 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_TYP_VORW_BED_1
    iCount = iCount + 1                 ' nächster Par
    
    ' Wert der 1.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(VAL_VORW_BED_1 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(VAL_VORW_BED_1 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_WERT_VORW_BED_1
    iCount = iCount + 1                 ' nächster Par
  
    ' Verknüpfung 1. und 2. Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(AND_OR_VORW_12 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(AND_OR_VORW_12 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = C_OR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_VKN_VORW_BED_12
    iCount = iCount + 1                 ' nächster Par
    
    ' Typ der 2.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(TYP_VORW_BED_2 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(TYP_VORW_BED_2 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_TYP_VORW_BED_2
    iCount = iCount + 1                 ' nächster Par
    
    ' Wert der 2.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(VAL_VORW_BED_2 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(VAL_VORW_BED_2 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_WERT_VORW_BED_2
    iCount = iCount + 1                 ' nächster Par
  
    ' Verknüpfung 2. und 3. Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(AND_OR_VORW_23 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(AND_OR_VORW_23 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = C_OR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_VKN_VORW_BED_23
    iCount = iCount + 1                 ' nächster Par
    
    ' Typ der 3.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(TYP_VORW_BED_3 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(TYP_VORW_BED_3 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_TYP_VORW_BED_3
    iCount = iCount + 1                 ' nächster Par
    
    ' Wert der 3.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(VAL_VORW_BED_3 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(VAL_VORW_BED_3 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_WERT_VORW_BED_3
    iCount = iCount + 1                 ' nächster Par
    
  ' bg019 max 5 Kriterien
    
    ' Verknüpfung 3. und 4. Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(AND_OR_VORW_34 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(AND_OR_VORW_34 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = C_OR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_VKN_VORW_BED_34
    iCount = iCount + 1                 ' nächster Par
    
    ' Typ der 4.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(TYP_VORW_BED_4 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(TYP_VORW_BED_4 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_TYP_VORW_BED_4
    iCount = iCount + 1                 ' nächster Par
    
    ' Wert der 4.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(VAL_VORW_BED_4 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(VAL_VORW_BED_4 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_WERT_VORW_BED_4
    iCount = iCount + 1                 ' nächster Par
  
    ' Verknüpfung 4. und 5. Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(AND_OR_VORW_45 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(AND_OR_VORW_45 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = C_OR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_VKN_VORW_BED_45
    iCount = iCount + 1                 ' nächster Par
    
    ' Typ der 5.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(TYP_VORW_BED_5 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(TYP_VORW_BED_5 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_TYP_VORW_BED_5
    iCount = iCount + 1                 ' nächster Par
    
    ' Wert der 5.Vorwärts - Bedingung
    stPar(iCount).yPropLo = FnLoByteOfLong(VAL_VORW_BED_5 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).yPropHi = FnHiByteOfLong(VAL_VORW_BED_5 + (iLoop * LAENGE_ABSCHN))
    stPar(iCount).lValue = NOT_PAR
    stPar(iCount).iScale = SCALE_ENUM
    stPar(iCount).strText = TXT_WERT_VORW_BED_5
    iCount = iCount + 1                 ' nächster Par
    
    
Next iLoop



'----------- Ende des Datensatzes -------------------------------------'
  stPar(iCount).yPropHi = END_PAR
  stPar(iCount).yPropLo = END_PAR
  stPar(iCount).bWrite = False

End Sub                          ' Ende InitParArray


