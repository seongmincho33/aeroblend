@echo off
setlocal

set BUILD_DIR=%1
if "%BUILD_DIR%"=="" set BUILD_DIR=build

echo === AeroBlend Native Build ===

echo --- CMake configure ---
cmake -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 (
    echo CMake configure failed.
    exit /b 1
)

echo --- CMake build ---
cmake --build "%BUILD_DIR%" --config Release --parallel
if errorlevel 1 (
    echo CMake build failed.
    exit /b 1
)

echo === Build complete ===
echo Run: %BUILD_DIR%\cpp\Release\aeroblend.exe --model ^<path-to-model.glb^>
