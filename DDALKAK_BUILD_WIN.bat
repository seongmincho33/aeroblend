@echo off
chcp 65001 >nul 2>&1
cd /d "%~dp0"

echo =========================================
echo   AeroBlend Docker 빌드 (Windows)
echo   처음엔 Rust+C++ 컴파일로 오래 걸림
echo =========================================
echo.

docker compose build

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [오류] 빌드 실패. Docker Desktop이 실행 중인지 확인하세요.
    pause
    exit /b 1
)

echo.
echo =========================================
echo   빌드 완료! DDALKAK_WIN.bat 으로 실행
echo =========================================
pause
