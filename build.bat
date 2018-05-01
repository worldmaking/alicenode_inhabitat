@if "%_echo%"=="" echo off 
@setlocal
set ALICE_DIR=%1
if not defined ALICE_DIR set ALICE_DIR="..\alicenode"
SET VSCMD_START_DIR=%cd%
SET VCVARS64=VC\Auxiliary\Build\vcvars64.bat
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\Enterprise\%VCVARS64%" call "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\Enterprise\%VCVARS64%"
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\Professional\%VCVARS64%" call "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\Professional\%VCVARS64%"
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\Community\%VCVARS64%" call "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\Community\%VCVARS64%"
if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\BuildTools\%VCVARS64%" call "%ProgramFiles(x86)%\Microsoft Visual Studio\2017\BuildTools\%VCVARS64%"

REM compile & link:
cl /nologo /LD /W3 /EHsc /O2 /I "%ALICE_DIR%\include" /I "%KINECTSDK20_DIR%\inc" "%ALICE_DIR%\alice.lib" "%ALICE_DIR%\lib\win64\openvr_api.lib" project.cpp user32.lib kernel32.lib shell32.lib gdi32.lib opengl32.lib "%KINECTSDK20_DIR%\lib\x64\kinect20.lib"

rem "%ALICE_DIR%\lib\win64\SpoutLibrary.lib" 

IF %ERRORLEVEL% NEQ 0 (
	echo ECHO compile/link failed with return code %ERRORLEVEL%
	EXIT /B %ERRORLEVEL%
)

@del project.obj project.exp

:end
@endlocal

