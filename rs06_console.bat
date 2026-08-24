@echo off
rem ---------------------------------------------------------------------------
rem  RS06 brake actuator console launcher
rem
rem  Double click to open the GUI, or from a prompt:
rem      rs06_console.bat            pick the port in the GUI
rem      rs06_console.bat COM3       connect to that port on startup
rem      rs06_console.bat --selftest check the Python environment, no GUI
rem
rem  Works from any current directory: it cds to its own folder first, so the
rem  relative path to tools\ and the logs\ output folder always resolve.
rem ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"

set "SCRIPT=tools\rs06_console.py"
if not exist "%SCRIPT%" goto :noscript

rem --- find an interpreter: plain python first, then the py launcher --------
set "PY="
where python >nul 2>&1
if %errorlevel% equ 0 set "PY=python"

if defined PY goto :run
where py >nul 2>&1
if %errorlevel% equ 0 set "PY=py -3"

:run
if not defined PY goto :nopython

%PY% "%SCRIPT%" %*
set "RC=%errorlevel%"
if not "%RC%"=="0" goto :failed
endlocal
exit /b 0

rem ---------------------------------------------------------------------------
:failed
echo.
echo === rs06_console exited with code %RC% ===
echo.
echo If a module is missing, install the dependencies with:
echo     %PY% -m pip install pyserial matplotlib numpy
echo.
echo To check the environment without opening the GUI:
echo     "%~nx0" --selftest
echo.
pause
exit /b %RC%

:nopython
echo.
echo Python 3 was not found on PATH.
echo Install it from https://www.python.org/downloads/ and tick
echo "Add python.exe to PATH" during setup, then run this again.
echo.
pause
exit /b 1

:noscript
echo.
echo Cannot find "%SCRIPT%" under "%~dp0".
echo Keep this batch file in the project root, next to the tools\ folder.
echo.
pause
exit /b 1
