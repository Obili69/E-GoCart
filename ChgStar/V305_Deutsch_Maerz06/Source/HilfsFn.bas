Attribute VB_Name = "HilfsFunktionen"
'---------------------------------------------------------------------------------------------------------------'
' MODUL NAME                          / Hilfsfunktionen
'---------------------------------------------------------------------------------------------------------------'
' MODUL BESCHREIBUNG                  / Stellt allgemeine Hilfsfunktionen, Konstanten etc. zur Verfügung, die   '
'                                     / oft gebraucht werden und NICHT gerätespezifisch sind.                   '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' HISTORY   / Datum     / Autor       / Kommentar                                                               '
'---------------------------------------------------------------------------------------------------------------'
' erstellt  / 15.11.01  / Beat G.     / Min / Max - Funktionen                                                  '
' geändert  / 11.12.01  / Beat G.     / Limit - Funktion und Byte <-> Long Konvertierungen                      '
'                                     /                                                                         '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
' FUNKTIONS - LISTE                   / Beschreibung                                                            '
'---------------------------------------------------------------------------------------------------------------'
' FnMin                               / von 2 Werten den kleineren bestimmen                                    '
' FnMax                               / von 2 Werten den grösseren bestimmen                                    '
' FnLimit                             / Variable auf einen Wert zwischen unterem und oberem Limit begrenzen     '
' FnLongOf2Bytes                      / Macht aus High- und Low- Byte einen Long                                '
' FnLoByteOfLong                      / Low Byte eines Long - Werts (0h..FFFFh) bestimmen                       '
' FnHiByteOfLong                      / High Byte eines Long - Werts (0h..FFFFh) bestimmen                      '
' FnGetCharOfNumber                   / Einzelne Ziffern einer Zahl lesen (z.B. Status LEDs)                    '
' PutCharToNumber                     / Einzelne Ziffern in einer Zahl setzen (z.B. Status LEDs)                '
' ConvertNumberToLocaleString         / Zahl einlesen, String mit Dezimaltrenner "." ohne 1000er Trenner retour '
' CompressTextToMaxLength             / Text (v.a. für Pfadnamen) in der Mitte kürzen, falls länger als erlaubt '
' ExtractFileNameFromPathFileString   / Dateinamen aus String mit Pfad- und Dateinamen extrahieren              '
'                                     /                                                                         '
'---------------------------------------------------------------------------------------------------------------'
'
'
'
'

Option Explicit

'--- Funktionshaufen, um lokale Einstellungen (z.B. Dezimaltrennzeichen) abzufragen, von VBnet Copy-Pasted ---


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Copyright ©1996-2002 VBnet, Randy Birch, All Rights Reserved.
' Some pages may also contain other copyrights by the author.
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Distribution: You can freely use this code in your own
'               applications, but you can not publish
'               or reproduce this code on any web site,
'               on any online service, or distribute on
'               any media without express permission.
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
Public Const LOCALE_SDECIMAL              As Long = &HE     'decimal separator
Public Const LOCALE_STHOUSAND             As Long = &HF     'thousand separator
Public Const LOCALE_SGROUPING             As Long = &H10    'digit grouping
Public Const LOCALE_IDIGITS               As Long = &H11    'number of fractional digits
Public Const LOCALE_ILZERO                As Long = &H12    'leading zeros for decimal
Public Const LOCALE_INEGNUMBER            As Long = &H1010  'negative number mode
Public Const LOCALE_SNATIVEDIGITS         As Long = &H13    'native ASCII 0-9
Public Const LOCALE_SPOSITIVESIGN         As Long = &H50    'positive sign
Public Const LOCALE_SNEGATIVESIGN         As Long = &H51    'negative sign

Public Const TEXT_LENGTH_FILLFACTOR       As Single = 0.92    ' "Füllfaktor" für Textbereich (Schaltflächen etc.)
Public Const TEXT_SHORTENING_FACTOR       As Single = 0.95   ' Kürzung um 5% bei jeder Iteration


Public Declare Function GetThreadLocale Lib "kernel32" () As Long

Public Declare Function GetSystemDefaultLCID Lib "kernel32" () As Long

Public Declare Function GetUserDefaultLCID Lib "kernel32" () As Long

Public Declare Function GetLocaleInfo Lib "kernel32" _
   Alias "GetLocaleInfoA" _
  (ByVal Locale As Long, _
   ByVal LCType As Long, _
   ByVal lpLCData As String, _
   ByVal cchData As Long) As Long


Public Function GetUserLocaleInfo(ByVal dwLocaleID As Long, ByVal dwLCType As Long) As String

   Dim sReturn As String
   Dim r As Long

  'call the function passing the Locale type
  'variable to retrieve the required size of
  'the string buffer needed
   r = GetLocaleInfo(dwLocaleID, dwLCType, sReturn, Len(sReturn))
    
  'if successful..
   If r Then
    
     'pad the buffer with spaces
      sReturn = Space$(r)
       
     'and call again passing the buffer
      r = GetLocaleInfo(dwLocaleID, dwLCType, sReturn, Len(sReturn))
     
     'if successful (r > 0)
      If r Then
      
        'r holds the size of the string
        'including the terminating null
         GetUserLocaleInfo = Left$(sReturn, r - 1)
      
      End If
   
   End If
    
End Function


'--ende Funktionshaufen --'









'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Funktion gibt den kleineren von 2 Werten zurück                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function FnMin(vArg1, vArg2 As Variant) As Variant
  If (vArg1 <= vArg2) Then
    FnMin = vArg1                  ' Argument1 ist das Minimum
  Else
    FnMin = vArg2                  ' Argument2 ist das Minimum
  End If
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Funktion gibt den grösseren von 2 Werten zurück                                                               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function FnMax(vArg1, vArg2 As Variant) As Variant
  If (vArg1 >= vArg2) Then
    FnMax = vArg1                  ' Argument 1 ist das Maximum
  Else
    FnMax = vArg2                  ' Argument 2 ist das Maximum
  End If
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' begrenzt den eingegebenen Wert auf den Bereich zwischen Low Limit und High Limit                              '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

' N.B.:  Mit "Hi" ist der grössere Wert, nicht der grössere Betrag gemeint (bei negativen Zahlen: "-25" > "-125")

Public Function FnLimit(vInVal, vLoLimit, vHiLimit As Variant) As Variant
  FnLimit = FnMax(vInVal, vLoLimit) ' Begrenzung gegen unten
  FnLimit = FnMin(FnLimit, vHiLimit) ' Begrenzung gegen oben
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Funktion macht aus Hi- und Lo-Byte einen Long Wert im Bereich 0..65535 (0h..FFFFh)                            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function FnLongOf2Bytes(yHiByte, yLoByte As Byte) As Long
  FnLongOf2Bytes = SHIFT_8_LEFT * yHiByte      ' High Byte "um 8 nach links schieben"
  FnLongOf2Bytes = FnLongOf2Bytes + yLoByte    ' Low Byte dazuzählen
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Extrahiert aus einem Long Wert das Low Byte                                                                   '
'                                                                                                               '
' lLongIn: 0..65535 (0h..FFFFh)  grössere Werte werden auf diesen Bereich begrenzt! (-> kein Absturz, aber..    '
'..unsinnige Resultate)                                                                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function FnLoByteOfLong(lLongIn As Long) As Byte
  Dim yHiByte As Byte
  yHiByte = FnHiByteOfLong(lLongIn)                                                ' Long "um 8 nach rechts schieben"
  FnLoByteOfLong = FnLimit(Fix(lLongIn - (SHIFT_8_LEFT * yHiByte)), 0, MAX_BYTE)   ' Rest, wenn man HiByte um 8 nach links schiebt und abzieht
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Extrahiert aus einem Long Wert das High Byte                                                                  '
'                                                                                                               '
' lLongIn: 0..65535 (0h..FFFFh)  grössere Werte werden auf diesen Bereich begrenzt! (-> kein Absturz, aber..    '
'..unsinnige Resultate)                                                                                         '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function FnHiByteOfLong(lLongIn As Long) As Byte
  FnHiByteOfLong = FnLimit(Fix(lLongIn / SHIFT_8_LEFT), 0, MAX_BYTE)               ' High Byte "um 8 nach rechts schieben"
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Einzelne Ziffer einer Dezimal - Zahl lesen                                                                    '
'                                                                                                               '
' lLongIn                       : Zahl, von der eine Ziffer bestimmt werden soll (Bereich: 0..65535)            '
' yFromRight                    : Stellenwert der Ziffer (von rechts her mit 0 beginnend gezählt)               '
' Rückgabe                      : die gesuchte Ziffer als Byte - Wert                                           '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function FnGetCharOfNumber(ByVal lLongIn As Long, ByVal iFromRight As Integer) As Byte
  
  Dim lLocVal As Long                                   ' lokale Kopie der Zahl
  Dim lAux As Long                                      ' Aux Value
  Dim yCount As Byte                                    ' Zähler
  Dim yFromRight As Byte                                ' Hilfsvar
  
  yCount = 0                                            ' Zähler initialisieren
  lLocVal = FnLimit(lLongIn, 0, MAX_UINT)               ' begrenzen
  yFromRight = FnLimit(iFromRight, 0, 4)                ' bei 0..65535 gibt's die Stellenwerte 0..4
  
  While (yCount <= yFromRight)
    lAux = Fix(lLocVal / 10)
    FnGetCharOfNumber = lLocVal - (10 * lAux)
    lLocVal = lLocVal / 10                              ' eine Dezimal - Stelle nach rechts
    yCount = yCount + 1                                 ' eins weiter
  Wend
  
  lLocVal = yCount
  
End Function



'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Einzelne Ziffer einer Dezimal - Zahl auf einen bestimmten Wert setzen                                         '
'                                                                                                               '
' lLongIn                       : Zahl, von der eine Ziffer gesetzt werden soll (Bereich: 0..65535)             '
' yChar                         : zu setzende Dezimal - Ziffer, also 0..9                                       '
' yFromRight                    : Stellenwert der Ziffer (von rechts her mit 0 beginnend gezählt)               '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

' Prinzip: Wert einer Stelle ermitteln, Differenz zum Soll bilden, Differenz abzählen

Public Sub PutCharToNumber(ByRef lLongIn As Long, ByVal yChar As Byte, ByVal iFromRight As Integer)
  
  Dim lLocVal As Long                                   ' lokale Kopie der Zahl
  Dim lAux As Long                                      ' Aux Value
  Dim lDiff As Long                                     ' Differenz
  Dim lRest As Long                                     ' Divisions - Rest
  Dim yCount As Byte                                    ' Zähler
  Dim yFromRight As Byte                                ' Hilfsvar
  
  yCount = 0                                            ' Zähler initialisieren
  lLocVal = FnLimit(lLongIn, 0, MAX_UINT)               ' begrenzen
  yFromRight = FnLimit(iFromRight, 0, 4)                ' bei 0..65535 gibt's die Stellenwerte 0..4
  
  While (yCount <= yFromRight)
    lAux = Fix(lLocVal / 10)
    lRest = lLocVal - (10 * lAux)                       ' Rest der 10er Division, also entspr. Stelle
    lDiff = yChar - lRest
    If (yCount = yFromRight) Then
      lLongIn = lLongIn + (10 ^ yCount * lDiff)
    End If
    lLocVal = lLocVal / 10                              ' eine Dezimal - Stelle nach rechts
    yCount = yCount + 1                                 ' eins weiter
  Wend
  
  lLocVal = yCount
  
End Sub





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Zahl als String einlesen und als String mit "." Dezimaltrenner, ohne 1000erTrenner zurückgeben                '
' bg023 Bugfix, Funktion erzeugt Konfusionen mit dem Dezimaltrennzeichen auf Win2000, wird hier eigentlich nur..'
'..dafür gebraucht, um Dezimalbrüche sicher mit "." als Trennzeichen zu speichern. Deshalb wird die Funktion..  '
'..umgebaut, dass sie nur noch das tut, aber dafür korrekt.                                                     '
'                                                                                                               '
' strNbr                        : umzuwandelnde Zahl (Bereich: Float)                                           '
' lSrcID                        : aktuelle Locale ID (Gebiets-/Sprach-Code gemäss Windows-Konvention)           '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function ConvertNumberToLocaleString(ByVal strNbr As String, ByVal lSrcID As Long) As String

Dim iCount As Integer                   ' Zählvariable
Dim strSrcDecimal As String             ' Dezimaltrenner auf diesem System
Dim strSrcThousand As String            ' 1000er Trenner auf diesem System
Dim strDstDecimal As String             ' Dezimaltrenner im Ziel-Format
Dim strDstThousand As String            ' 1000er Trenner im Ziel-Format
Dim strAux As String                    ' Hilfs - String


  'Idee: Dezimaltrennzeichen, separatoren und solchen Plunder feststellen für aktuelle Locale,..
  '..dann Zeichen austauschen - fertig!
  'CStr und so wie jetzt in aufrufender Funktion; lesen/schreiben von/nach Datei, alle Bereiche abklappern!
  
    strSrcDecimal = GetUserLocaleInfo(lSrcID, LOCALE_SDECIMAL)      ' aktuelle Zeichen feststellen
    strSrcThousand = GetUserLocaleInfo(lSrcID, LOCALE_STHOUSAND)
    strDstDecimal = "."                                             ' Dezimaltrennzeichen ist immer Punkt
    strDstThousand = ""                                             ' Tausendertrennzeichen entfernen
                                                                        
    If (strDstThousand <> strSrcThousand) Then
      For iCount = 1 To Len(strNbr)
        If Mid(strNbr, iCount, 1) = strSrcThousand Then             ' 1000er Trenner entfernen (v.a. Punkte wären schlimm)
          Mid(strNbr, iCount, 1) = strDstThousand
        End If
      Next iCount
    End If
    
    If (strDstDecimal <> strSrcDecimal) Then
      For iCount = 1 To Len(strNbr)
        If Mid(strNbr, iCount, 1) = strSrcDecimal Then                ' Dezimaltrenner ersetzen
          Mid(strNbr, iCount, 1) = strDstDecimal
        End If
      Next iCount
    End If
  
  
  ConvertNumberToLocaleString = strNbr                                  ' korrigierten String zurückgeben

End Function





'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' String so kürzen, dass er in einem gegebenen Textfeld sicher Platz hat und Anfang und Ende erhalten bleiben   '
'                                                                                                               '
' strText                       : zu prüfender / kürzender Text                                                 '
' lMaxLength                    : max. erlaubte Länge des Texts in Bildschirm (Pixel) Einheiten                 '
' objDisplay                    : Objekt, das den Text anzeigen soll                                            '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function CompressTextToMaxLength(ByVal strText As String, ByVal lMaxLength As Long, ByVal objDisplay As Object) As String

Dim iAux As Integer                     ' Hilfs Int
Dim iNumChar As Integer                 ' Anzahl Zeichen
Dim lActLength As Long                  ' aktuelle "Pixel"-Länge des Strings
Dim strLeft As String                   ' linker Teilstring
Dim strRight As String                  ' rechter Teilstring
Dim strAux As String                    ' Hilfs - String



  'Idee:
  ' 1.  Länge von strText bestimmen: wenn kleiner als MaxLength, gib strText zurück, sonst kürze in der Mitte..
  '     ..die Anzahl Zeichen, jeweils um (z.B. 5) Prozent, und fülle Punkte ein
  ' 2.  Gehe zu 1.
  
  strAux = strText                                ' Rückgabewert mit übergebenem Text initialisieren
  lActLength = objDisplay.TextWidth(strAux)       ' aktuelle Länge des Strings bestimmen
  iNumChar = Len(strAux)                          ' Anzahl Zeichen bestimmen
  
  
   iAux = objDisplay.TextWidth("ABCDEFGHIJK")     ' schätzative Länge von Icon + Max/Min/Close Buttons + Abstände
'   strAux = objDisplay.Caption
  
  While (lActLength > (TEXT_LENGTH_FILLFACTOR * lMaxLength))                      ' String ist länger als erlaubt
    strLeft = Left(strAux, ((iNumChar * TEXT_SHORTENING_FACTOR * 0.55) - 1))      ' linke Hälfte kürzen
    strRight = Right(strAux, ((iNumChar * TEXT_SHORTENING_FACTOR * 0.45) - 1))    ' rechte Hälfte kürzen
    strAux = strLeft & "..." & strRight                                           ' String neu zusammensetzen
    lActLength = objDisplay.TextWidth(strAux)     ' aktuelle Länge des Strings bestimmen
    iNumChar = Len(strAux)                        ' Anzahl Zeichen bestimmen
  Wend
  
  CompressTextToMaxLength = strAux                ' Rückgabewert = komprimierter Text

End Function






'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
' Datei - Name aus einem String mit Datei- und Pfadname herauspflücken, da das VB-Objekt "FileSystemObj" o.ä..  '
'..diesen nicht separat führt. Im Prinzip einfach lesen von rechts, bis ein Backslash auftritt.                 '
'                                                                                                               '
' strFilePathName               : Datei- und Pfadname                                                           '
'                                                                                                               '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Public Function ExtractFileNameFromPathFileString(ByVal strFilePathName As String) As String

Dim lAux As Long                        ' Hilfs - Long
Dim strParts() As String                ' Array von Strings, für die Auswertung von Split
  
                                        ' Quellstring in Teilstrings aufteilen, Trennzeichen ist Backslash
  strParts = Split(strFilePathName, "\", -1, vbTextCompare)
  lAux = UBound(strParts)               ' gesuchter Dateiname ist letzter String im Array
  ExtractFileNameFromPathFileString = strParts(lAux)    ' Rückgabewert = "nackter" Datei-Name

End Function

