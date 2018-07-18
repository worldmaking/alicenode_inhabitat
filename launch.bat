echo off
title Inhabitat Launcher
echo "Insuperposition Launcher"

:: launch max and audiostate patcher  
rem start ../alicenode_inhabitat/audio/audiostate_sonification.maxpat &
echo "Launching Max/MSP & Sonification Patch on process ID $!" &

:: Start Alice!
echo Starting Alice!
cd ..\alicenode_inhabitat
..\alicenode\alice.exe project.dll
echo Exit Code is %errorlevel%

pause
