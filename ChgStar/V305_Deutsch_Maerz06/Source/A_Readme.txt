Datei A_Readme.txt						18.11.04 bg

Source Code ChargeStar:

In diesem Verzeichnis sind alle Basic-Quelldateien, Projekt-Dateien usw. enthalten, die unmittelbar in Basic geschrieben wurden. Um daraus ein kompilierbares und lauffähiges Projekt zu erstellen, braucht es aber noch weitere Standard- oder Bibliotheks-Komponenten. Diese müssen in der Regel auf dem betreffenden Entwicklungsrechner installiert sein:

- MS Visual Basic 6 bzw. MS Visual Studio 6
- National Instruments Component Works (>= V3.0?), z.B. aus Measurement Studio
- OCX'en / DLL's, die üblicherweise mit Windows '98 bereits installiert sind, wie comdlg, help usw.

Eine detaillierte Liste aller benötigten Dateien ist in der Datei Setup.lst der Release Version ersichtlich.
Zum "vertriebsfertigen" Kompilieren und Packen der Anwendung wird am besten der "Paket- und Weitergabe-Assistent" von Visual Basic benutzt, mit dem entsprechenden Verpackungsskript.

Verschiedene Sprachen: in der Datei "DatenKapselUndKonstanten.bas" gibt es einen 'Schalter' namens g_lLocaleOffset für die Sprache, der wird mittels Konstante LOCALE_DEUTSCH bzw. LOCALE_ENGLISH gesetzt.

Das genaue Vorgehen für die Erstellung neuer Versionen ist in der Software Dokumentation zu ChargeStar NLG 5 beschrieben.