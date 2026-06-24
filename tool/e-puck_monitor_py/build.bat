@echo off
REM Build e-puck Monitor standalone package
REM All build artifacts stay in this directory - nothing goes to C:

setlocal
set "APP_DIR=%~dp0"
set "DIST_DIR=%APP_DIR%dist"
set "BUILD_DIR=%APP_DIR%build"

REM Redirect PyInstaller internal cache away from C: drive
set "PYINSTALLER_CONFIG_DIR=%APP_DIR%.pyinstaller_cache"

echo === Cleaning previous build ===
if exist "%DIST_DIR%" rmdir /s /q "%DIST_DIR%"
if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
if exist "%PYINSTALLER_CONFIG_DIR%" rmdir /s /q "%PYINSTALLER_CONFIG_DIR%"

echo === Building e-puck Monitor ===
python -m PyInstaller ^
    --onedir ^
    --windowed ^
    --name "e-puck Monitor" ^
    --distpath "%DIST_DIR%" ^
    --workpath "%BUILD_DIR%" ^
    --specpath "%APP_DIR%" ^
    --add-data "%APP_DIR%README.txt;." ^
    --clean ^
    --noconfirm ^
    "%APP_DIR%epuck_monitor.py"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo === BUILD FAILED ===
    pause
    exit /b 1
)

echo.
echo === Build successful ===
echo Output: %DIST_DIR%\e-puck Monitor\
echo.
echo === Copying documentation ===
if exist "%APP_DIR%README.txt" copy /y "%APP_DIR%README.txt" "%DIST_DIR%\e-puck Monitor\" >nul

echo.
echo === Creating portable package ===
powershell -Command "Compress-Archive -Path '%DIST_DIR%\e-puck Monitor' -DestinationPath '%DIST_DIR%\e-puck-Monitor-portable.zip' -Force"

echo.
echo === Done ===
echo Portable folder: %DIST_DIR%\e-puck Monitor\
echo Portable zip:    %DIST_DIR%\e-puck-Monitor-portable.zip
echo.
echo To run: "%DIST_DIR%\e-puck Monitor\e-puck Monitor.exe"
pause
