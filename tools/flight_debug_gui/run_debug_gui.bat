@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if %errorlevel%==0 (
  set "PY=py -3"
) else (
  set "PY=python"
)

%PY% -c "import tkinter" >nul 2>nul
if not %errorlevel%==0 (
  echo Python tkinter is not available. Please install a normal Python for Windows.
  pause
  exit /b 1
)

%PY% -c "import serial" >nul 2>nul
if not %errorlevel%==0 (
  echo Installing pyserial...
  %PY% -m pip install -r requirements.txt
  if not %errorlevel%==0 (
    echo Failed to install pyserial. Try: python -m pip install pyserial
    pause
    exit /b 1
  )
)

%PY% flight_debug_gui.py
pause
