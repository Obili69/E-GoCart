Attribute VB_Name = "UnivSerCom"
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / Universelle Serielle Kommunikation                                                          '
'---------------------------------------------------------------------------------------------------------------'
' MODUL BESCHREIBUNG                  / Funktionen für Kommunikation über RS232, (ev. auch andere). Momentan    '
'                                     / ist nur RS232 implementiert. Diese Funktionen sind universell.          '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' HISTORY   / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' ERSTELLT  / März '02  / Beat G.     / Universalfunktionen aus SeriellKomm entfernt und hier platziert         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
' ConnectNLG                          / Verbindung mit NLG aufbauen (einloggen etc.)                            '
' SendReceiveProfile                  / Kennlinie versenden oder empfangen                                      '
' WrapString                          / Datenstrom mit Länge, CRC usw. versandbereit machen                     '
' CalculateCRC                        / Checksumme des zu sendenden Datenstroms berechnen                       '
' SendFrame                           / Datenstrom (Frame) byteweise an der seriellen Schnittstelle absetzen    '
' WaitForReply                        / Antwort von NLG auf versendetes Frame empfangen und auswerten           '
' FnGetByteFromFrame                  / ein bestimmtes Byte aus einem Frame holen                               '
' CheckFrameLengthAndCRC              / Länge und Checksumme eines Frames auf Korrektheit prüfen                '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'
'
' Achtung:  Mangels sinnvoller Alternativen wurde die Datenhaltung des Sende - Streams mit einem globalen Array
'           yOutStream realisiert; dito CRC
' Und es geht noch weiter: da das Kommunikationsobjekt am entsprechenden Formular (z.B. Kennlinien - Programm)..
'..hängt, wird die Empfangsfunktion "MSComm1_OnComm" auch dort platziert. Damit sich die Daten aber nicht beliebig..
'..verstreuen und duplizieren, bleiben die entsprechenden Arrays hier im Universalmodul (sozusagen als Schnittstelle)
' Im Formular gibt es auch eine Funktion "Poll_Send", welche solange "Halb-aktiv kooperativ" wartet, bis das Frame..
'..an der seriellen Schnittstelle des PC vollständig abgesetzt ist
'
'
Option Explicit

' rrr diese voll globalen Variablen müssen später einigermassen privat oder wenigstens modulglobal werden
Public yOutStream(COM_MAX_LEN) As Byte      ' total globaler Array für den Out - Stream
Public yHiCRC As Byte, yLoCRC As Byte       ' total globale Variablen für CRC Hi und Lo Byte

' rrr "Schnittstelle" zur Empfangs - Funktion (Telefönli-Objekt, Comm1)
Public strInBuffer As String                ' String variabler Länge, hier kommen die empfangenen Daten rein
Public strRecFrameBuf As String             ' empfangenes, komplettes Frame zwischenspeichern zum Verarbeiten

Dim ret As Integer                          ' nur für Debug







'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Verbindung zum NLG aufbauen bzw. abbrechen, login, steuerbefehle absetzen etc. Das eigentliche Öffnen des..   '
'..Comm - Ports muss beim Objekt selber geschehen, also im Kennlinienprogramm                                   '
'                                                                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Sub ConnectNLG()
  
  Dim lVal    As Long                                   ' Rückgabewert
  Dim iRetry  As Integer                                ' Sende - Zähler
  Dim yHi     As Byte                                   ' Hi - Byte der Property
  Dim yLo     As Byte                                   ' Lo - Byte der Property
  Dim strAux  As String                                 ' Hilfs - String
  Dim bErr    As Boolean                                ' Fehler in der Kommunikation
  
                                                        ' Login - String erstellen
  yOutStream(0) = 112       'p
  yOutStream(1) = 114       'r
  yOutStream(2) = 111       'o
  yOutStream(3) = 102       'f
  yOutStream(4) = 105       'i
  yOutStream(5) = 108       'l
  yOutStream(6) = 101       'e
  yOutStream(7) = 13        'cr
  yOutStream(8) = 10        'lf
  yOutStream(COM_MAX_LEN) = 9
  
  Call SendFrame                                      ' Login - String senden
  
  
  ' allfälligen Ladevorgang stoppen
  bErr = True
  iRetry = 0                                          ' Sende - Zähler initialisieren
  yHi = FnHiByteOfLong(CHARGE_START_STOP)             ' Hi - Byte des Start/Stop Kommandos
  yLo = FnLoByteOfLong(CHARGE_START_STOP)             ' Lo - Byte des Start/Stop Kommandos
  strAux = Chr$(0) & Chr$(COM_STOP_CHG)               ' Wert = Stop
  Call WrapString(OP_SET, yHi, yLo, strAux)           ' Stop - Befehl zum Senden aufbereiten

  While ((bErr = True) And (iRetry < ERR_RETRY))      ' ggf. mehrere Versuche
    Call SendFrame                                    ' Stop - Befehl senden
    Call WaitForReply(OP_GET, yHi, yLo, lVal, bErr)   ' Antwort von NLG abfragen / abwarten
    iRetry = iRetry + 1                               ' Anz. Sendeversuche inkrementieren
  Wend
  
  strAux = strAux & Chr$(69)                          ' Dummy Instruktion

End Sub                                               ' Ende ConnectNLG





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden oder Empfangen der ganzen Kennlinie. Will heissen, vorgegebenen Array mit allen  Parametern ..         '
'..durchzählen (Datenkapsel füllen / auslesen), einen Befehl senden, auf Quittierung warten, ggf. Befehl..      '
'..wiederholen.                                                                                                 '
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
  
  
  
'  If MSComm1.PortOpen = True Then                     ' serielle Kommunikation offen?
    
  yHiPrev = NOT_PAR                                   ' Am Anfang ist der "vorherige Par" der "NichtParameter" an Stelle 0
  yLoPrev = NOT_PAR                                   '
    
                              ' ------------- alle Parameter durchzählen --------------
  While (GetNextPar(yHiPrev, yLoPrev, yHi, yLo) = True)
    
    iErr = 0                                          ' Fehler - Zähler reseten
    
    lVal = GetDataFromCapsule(yHi, yLo)               ' Wert des aktuellen Parameters holen
    
    strTmp = Chr(FnHiByteOfLong(lVal))                ' High Byte als String
    strTmp = strTmp & Chr(FnLoByteOfLong(lVal))       ' Low Byte als String anhängen
        
    If bSet = True Then                   '--------- Werte Setzen/Kennlinie senden---------- ?
        
      If (GetStatusOfPar(yHi, yLo) = True) Then       ' Darf der Wert des Parameters überschrieben werden?
        Call WrapString(OP_SET, yHi, yLo, strTmp)     ' Frame erzeugen
        Do                                            ' versuche, ein Frame abzusetzen
          Call SendFrame
' rrr          Do                                          ' "Halb-Busy-Wait" bis Frame abgesetzt
            ret = DoEvents()                          ' Windows Ereignisse ausführen ("kooperatives Multitasking")
' rrr          Loop Until MSComm1.OutBufferCount = 0       ' Stream vollständig abgesetzt
          Call WaitForReply(OP_SET, yHi, yLo, lVal, bErr)     ' Antwort von NLG abfragen / abwarten
          If (bErr = True) Then                       ' Fehler-Zähler
            iErr = iErr + 1
          End If
        Loop Until ((bErr = False) Or (iErr > ERR_RETRY))     ' Frame korrekt abgesetzt? Sonst nochmals versuchen (max. 4-mal)
        If (iErr > ERR_RETRY) Then
          If (SWITCH_DEBUG = True) Then
            MsgBox ("Parameter setzen / lesen nicht möglich")
          End If
        End If
      End If                                          '
        
    Else                                  '----------- nicht setzen/senden, sondern abfragen --------
          
      strTmp = ""                                     ' beim Abfragen werden keine Daten mitgesendet
      Call WrapString(OP_GET, yHi, yLo, strTmp)
          Do                                          ' versuche, ein Frame abzusetzen
            Call SendFrame
' rrr            Do                                        ' "Halb-Busy-Wait" bis Frame abgesetzt
              ret = DoEvents()                        ' Windows Ereignisse ausführen ("kooperatives Multitasking")
' rrr            Loop Until MSComm1.OutBufferCount = 0     ' Stream vollständig abgesetzt
            Call WaitForReply(OP_GET, yHi, yLo, lVal, bErr)   ' Antwort von NLG abfragen / abwarten
          Loop Until (bErr = False)                   ' Frame korrekt abgesetzt? Sonst nochmals versuchen
             
    End If                                '----------- Ende setzen / abfragen -------------------------
    
    yHiPrev = yHi
    yLoPrev = yLo
  Wend                                  ' ------------- Ende Parameter durchzählen --------------
  
  'Else                                                ' Verbindung nicht offen, also warnen
  '  MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
  'End If

End Sub






'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Senden des fixfertigen Frames, d.h den in yOutStream enthaltenen Datenstrom Byte für Byte in den Output- ..   '
'..Puffer der RS232 schieben, "kooperativ" warten bis sie abgesetzt sind                                        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub SendFrame()
  
  Dim iCount As Integer                             ' lokale Zählvariable
  
    
  If MSComm1.PortOpen = True Then                   ' serielle Kommunikation offen?
    
    For iCount = 0 To (yOutStream(COM_MAX_LEN) - 1) ' ganzes Frame in den OutputBuffer
      MSComm1.Output = Chr(yOutStream(iCount))
    Next iCount
    
    Do                                              ' "Halb-Busy-Wait" bis Frame abgesetzt
      ret = DoEvents()                              ' Windows Ereignisse ausführen ("kooperatives Multitasking")
    Loop Until MSComm1.OutBufferCount = 0
  
    Call DispSendFrame                              ' rrr Debug, gesendetes Frame anzeigen
  
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If

End Sub




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
  
  ' Länge des Frames inkl. Overhead ermitteln, an 4.Stelle schreiben
  yOutStream(4) = iStringLen + COM_OVERHEAD
  
  ' Werte-String Byte für Byte in den Out - Stream schreiben, an der richtigen Stelle
  For iCount = 1 To iStringLen
    yOutStream(COM_POS_VAL_START - 1 + iCount) = Asc(Mid(strInString, iCount, 1))
  Next iCount
  
  ' Checksumme (CRC) über alle Bytes bis hierhin ermitteln und schreiben (im Moment nur Quersumme)
  Call CalculateCRC(iStringLen)                 ' Checksumme ermitteln
  
  yOutStream(COM_POS_VAL_START + iStringLen) = yHiCRC               ' High - Byte der Checksumme an drittletzter Stelle
  yOutStream(COM_POS_VAL_START + 1 + iStringLen) = yLoCRC           ' Low - Byte der Checksumme an zweitletzter Stelle
  yOutStream(COM_POS_VAL_START + 2 + iStringLen) = COM_ETX          ' End - Zeichen für Kommunikation

End Sub




'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Warten auf passende Antwort zum ausgesendeten Befehl; wenn korrekte Antwort, dann allfälliges Resultat..      '
'..weitergeben, sonst Fehler melden                                                                             '
'                                                                                                               '
' yOpID           : Operation, die auf diese Geräte - Eigenschaft angewendet wird: setzen, lesen, usw.          '
' yPropIDHi/Lo    : Geräte - Eigenschaft: Gerätetyp, Nennspannung der Batterie, Wechsel-Bedingung usw.          '
' lValue          : allfällig zurückgegebener Wert der Eigenschaft                                              '
' bError          : Flag, welches meldet, ob ein Fehler auftrat (True) oder korrekte Antwort kam (False)        '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Private Sub WaitForReply(yOpID As Byte, yPropIDHi As Byte, yPropIDLo As Byte, lValue As Long, bError As Boolean)
  
  Dim iCount As Integer                             ' lokale Zählvariable
  Dim yAuxByte() As Byte                            ' Zwischenspeicher für Daten
  Dim fTimeStamp As Single                          ' Zeitstempel für's Warten
  Dim bTimeOut As Boolean                           ' Flag für Timeout
  
  
  fTimeStamp = Timer                                ' Zeitstempel = aktuelle Systemzeit in Sekunden
  bError = True                                     ' der wird nur False gesetzt, wenn die Antwort 100%-ig richtig war
  
  If (MSComm1.PortOpen = True) Then                   ' serielle Kommunikation offen?
    
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
         
         Case OP_SET                                  ' auf "Set" müssen als Antwort OP_OK, Prop_ID's und Value=ACK folgen
           If (FnGetByteFromFrame(COM_POS_OP_ID, strRecFrameBuf) = OP_OK) Then            ' OP_OK ist ok; sind Prop_ID's auch korrekt?
             If ((FnGetByteFromFrame(COM_POS_PROP_HI, strRecFrameBuf) = yPropIDHi) And (FnGetByteFromFrame(COM_POS_PROP_LO, strRecFrameBuf) = yPropIDLo)) Then    ' Prop_ID Hi und Lo ok
               yAuxByte = FnGetByteFromFrame(COM_POS_VAL_START, strRecFrameBuf)           ' Daten abholen (könnten bei Fehler mehrere sein)
               If (yAuxByte(0) = COM_ACK) Then                                            ' Msg verstanden
                 bError = False                                                           ' ok
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
               Call PutDataToCapsule(yPropIDHi, yPropIDLo, lValue)                        ' empfangene Daten in die Kapsel schreiben
               bError = False                                                             ' ok
             End If
           End If
          
          Case Else                                                                       ' Alles Übrige
            If SWITCH_DEBUG = True Then
              MsgBox ("unbekannte OP_ID bei WaitForReply")
            End If
        
        End Select
      
      Else
        bError = True                                 ' Frame nicht ok
      End If                                          ' Ende Frame prüfen
    
    Else                                            ' keine rechtzeitige Antwort eingetroffen
      If SWITCH_DEBUG = True Then
        MsgBox ("Timeout bei Warten auf Antwort")
      End If
    End If                                          ' Ende rechtzeitige Antwort eingetroffen
    
  g_bMsgReceived = False                            ' empfangene Msg verarbeitet, also Msg löschen
  
  Else                                              ' Verbindung nicht offen, also warnen
    If SWITCH_DEBUG = True Then
      MsgBox ("Es besteht noch keine Verbindung oder Gerät nicht bereit")
    End If
  End If                                            ' Ende Kommunikation offen

End Sub                                             ' Ende WaitForReply

