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

rem /I "%KINECTSDK20_DIR%\inc" 
rem "%KINECTSDK20_DIR%\lib\x64\kinect20.lib"

rem lib/win64/zlib.lib lib/win64/ippicvmt.lib lib/win64/opencv_core320.lib lib/win64/opencv_flann320.lib lib/win64/opencv_calib3d320.lib lib/win64/opencv_imgproc320.lib lib/win64/opencv_features2d320.lib lib/win64/opencv_video320.lib lib/win64/opencv_tracking320.lib ^

REM compile & link:
cl /nologo /LD /W3 /EHsc /Ox /I "%ALICE_DIR%\include" "%ALICE_DIR%\alice.lib" "%ALICE_DIR%\lib\win64\openvr_api.lib" "%ALICE_DIR%\lib/win64/lib-vc2017/glfw3.lib" project.cpp "%ALICE_DIR%\lib/win64/ippicvmt.lib" "%ALICE_DIR%\lib/win64/zlib.lib"  "%ALICE_DIR%\lib/win64/opencv_core320.lib" "%ALICE_DIR%\lib/win64/opencv_imgproc320.lib" "%ALICE_DIR%\lib/win64/opencv_video320.lib" user32.lib kernel32.lib shell32.lib gdi32.lib opengl32.lib 

rem "%ALICE_DIR%\lib\win64\SpoutLibrary.lib" 

IF %ERRORLEVEL% NEQ 0 (
	echo ECHO compile/link failed with return code %ERRORLEVEL%
	EXIT /B %ERRORLEVEL%
)

@del project.obj project.exp

:end
@endlocal

