@echo off
where cl >nul 2>&1
if errorlevel 1 (
  for /f "usebackq delims=" %%V in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat`) do call "%%V" >nul
)
where cl >nul 2>&1
if errorlevel 1 (
  echo MSVC C compiler not found. Run from a Visual Studio x64 Native Tools prompt.
  exit /b 1
)
set PYTHONDONTWRITEBYTECODE=1
python "%~dp0run_tests.py"
exit /b %errorlevel%
