Attribute VB_Name = "INI_datei"
' Die "Private Profile String" Funktionen bilden den Schlüssel zu den INI - Files
Declare Function WritePrivateProfileString _
Lib "kernel32" Alias "WritePrivateProfileStringA" _
(ByVal lpApplicationname As String, ByVal _
lpKeyName As Any, ByVal lsString As Any, _
ByVal lplFilename As String) As Long

Declare Function GetPrivateProfileString Lib _
"kernel32" Alias "GetPrivateProfileStringA" _
(ByVal lpApplicationname As String, ByVal _
lpKeyName As String, ByVal lpDefault As _
String, ByVal lpReturnedString As String, _
ByVal nSize As Long, ByVal lpFileName As _
String) As Long
Function datei_exist(f_datei) As Boolean
    '''prüffen ob eine Datei existiert...
  
  Dim fs   ' vermutlich Variant
    Set fs = CreateObject("Scripting.FileSystemObject")
    datei_exist = fs.FileExists(f_datei)

End Function

Function ini_lesen(ini_name As String, ini_titel As String, ini_var As String) As String
    '''Daten lesen von eine INI Datei und die restlichen leerstellen bis 100 abschneiden...

Dim lngResult As Long
Dim strFileName
Dim strResult As String * 100


        lngResult = GetPrivateProfileString(ini_titel, _
        ini_var, ini_name, strResult, Len(strResult), _
        ini_name)

ll = InStr(1, strResult, Chr(0)) - 1
ini_lesen = Left(strResult, ll)



End Function

