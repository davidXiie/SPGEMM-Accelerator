@echo off
REM =============================================================================
REM File     : run.bat
REM Project  : SPGEMM-Accelerator
REM Brief    : Windows run script - generate data + run cocotb simulation
REM =============================================================================

echo ========================================
echo  SPGEMM-Accelerator Testbench
echo ========================================

REM Step 1: Generate test data
echo.
echo [1/2] Generating test data...
python gen_data.py
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: gen_data.py failed!
    pause
    exit /b 1
)

REM Step 2: Run cocotb simulation
echo.
echo [2/2] Running cocotb simulation...
echo   Test: test_spgemm_tc1
echo   DUT:  core_top
echo   SIM:  icarus
echo.

make TESTCASE=test_spgemm_tc1 SIM=icarus

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo WARNING: Simulation reported errors. Check logs.
) else (
    echo.
    echo ========================================
    echo  Simulation completed successfully!
    echo ========================================
)

pause
