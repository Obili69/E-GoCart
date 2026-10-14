
Readme file for BRUSA ChargeStar NLG5 V3.04, 		23.Dec.2004


Contents

1. Installation
 1.1 Requirements
 1.2 New installation
 1.3 Update from older version
2. Remove the program 
3. Troubleshooting
 3.1 Loading and saving profile files is not possible
 3.2 Connection to NLG does not work
 3.3 Firmware version is not compatible
 3.4 Decimal fractions saved incorrectly / wrong decimal separator displayed
4. What's new
5. Additional information

________________________________________

1. Installation

 1.1 Requirements

BRUSA ChargeStar NLG5 runs on Windows versions 98, Me, 2000 and XP. ChargeStar is delivered as self extracting archive, a file named "ChSt304E.exe". The firmware version of the NLG5 that is to be programmed must be at least V3.00.00 (may be checked under "Extras | Charger info", if the data connection NLG-PC is established).

 1.2 New installation

Close all open applications and double click on ChSt304E.exe. Follow the instructions of the assistant that will lead you through the extraction and installation process.

 1.3 Update from older version

First make a copy of your own profile files (*.pfl) to prevent losing them. You can either update the existing installation or install the new version in addition to the existing one (parallel installation).
For a parallel installation you have to specify a different path and program group for the new version.
For an update of a 1.x or 2.x version you can proceed exactly the same way as with a new installation (see 1.1). If you update from a 3.x version you should first uninstall the older version (see 2.).


2. Remove the program 

To remove ChargeStar from your PC, select "Software" from the control panel. Then select "BRUSA ChargeStar NLG5" and click on "Remove".


3. Troubleshooting

 3.1 Loading and saving profile files is not possible

When trying to load a profile from file, ChargeStar reports "...is not a valid Profile file"; when trying to save a profile to file, the program reports "ActiveX component can't create object". In this case proceed as follows: from the windows taskbar select "Start | settings | control panel | Software", click on the tab "Windows Setup" and open the submenu "Accessories". Activate the entry "Windows Scripting Host" and follow the instructions (if applicable). Close all windows by OK and restart the PC.

 3.2 Connection to NLG does not work

Although PC and NLG are wired correctly, the connection between them does not work. The following error message appears

  a) "Connection can't be established"
If connection is not successful after 2 retries, check the electric setup and the PC configuration (see below); restart the PC, then reset the NLG (switch off the power and then on again), then start ChargeStar.

   Wiring
For programming purposes the NLG can be powered from two different sources: either via the control connector from external 12VDC supply or via mains line from 100-240VAC. You can run the charger without connecting the traction battery, when powered from 12VDC there are no other precautions required. If it's powered from mains, then either the traction battery must be connected or the Power ON signal must be OFF, i.e. the jumper between Pin2 and 3 of the included control connector must be removed during programming (see user's manual).

   PC configuration
Check the settings of the PC's COM port, see "Start | settings | control panel | system | device manager | ports (COM and LPT) | Communications port (COM1) | resources"; it must be set to "basic configuration 0000" and "use automatic settings", the description must read "Communications Port (COM1)".

   b) "serial port is already in use by another application"
An application used COM1 port and didn't release it yet. Exit all other programs that (might) use COM1 like modems, terminal programs etc. and restart the PC if necessary.

   c) "serial port is not ready"
The COM1 port cannot be used because Windows reports it to be not functioning or not existing. Check PC configuration as stated above and restart the PC if necessary.

 3.3 Firmware version is not compatible
If the connection between PC and NLG can be established but a message appears "CAUTION: Firmware version of the NLG is not fully compatible...", then the firmware of the NLG5 does not support all functions of the new ChargeStar software. In this case you should either update the firmware of the NLG5 (for more information please contact BRUSA Elektronik AG) or continue using the older version of ChargeStar. We recommend not to use the new ChargeStar version to program NLG's with older firmware. If you decide to  do so anyway, you must acknowledge all warnings by "Ignore" and make sure that no functions are used which were not present in the old version (e.g. type and number of the conditions to change profile sections). A charger that was misprogrammed this way would not function correctly!

 3.4 Decimal fractions saved incorrectly / wrong decimal separator displayed
This can happen on a multiuser system like Win2000 or WinXP if the standard settings for regional options are not equal to the individual user settings. In order to get correct behaviour, select exactly the same locale/country under "Start | settings | control panel | regional options | your locale (location)" as standard like the one that is set under "Start | settings | control panel | regional options | input locales | input language" as standard. This one usually fits your keyboard and therefore should not be changed. If extended regional options are available, they must be set to the same language/country combination as above.


4.   What's New

ChargeStar V3.04 contains the following new features compared to V2.x:
- CAN parameters of the charger (baud rate and identifiers) are selectable
- extended online Help
- adaptions to regional settings of Win2000/WinXP
- setup adapted to Win2000/WinXP
- adaptions for NLG5 booster operation

ChargeStar V2.x contains the following new features compared to V1.0x:
- if the battery's nominal voltage is changed, ChargeStar adapts all related values to the new nominal voltage automatically, if desired
- max 5 conditions for section change over in the profile (used to be max. 3)
- new conditions for section change over like temperature rise, voltage rise and now also digital inputs of the NLG5
- under "Extras | Properties" a comment text for the profile is available
- if the NLG5 is connected and logged in, under "Extras | Properties | Print all properties" a summary of the charger and profile properties can be printed
- ChargeStar available in English and German
- extended online Help


5.   Additional information

Software, profiles, manuals, contact information and additional information is found on our website www.brusa.biz.
