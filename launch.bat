echo off
title Inhabitat Launcher
echo "Insuperposition Launcher"

rem start "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrmonitor.exe"

timeout 10

:: launch max and audiostate patcher  
start ../alicenode_inhabitat/audio/audiostate_sonification_nows.maxpat &
echo "Launching Max/MSP & Sonification Patch on process ID $!" &

timeout 10

:: Start Alice!
echo Starting Alice!
cd ..\alicenode_inhabitat
..\alicenode\alice.exe project.dll
echo Exit Code is %errorlevel%

pause
