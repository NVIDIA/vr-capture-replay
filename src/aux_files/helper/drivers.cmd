:: * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
:: *
:: * NVIDIA CORPORATION and its licensors retain all intellectual property
:: * and proprietary rights in and to this software, related documentation
:: * and any modifications thereto.  Any use, reproduction, disclosure or
:: * distribution of this software and related documentation without an express
:: * license agreement from NVIDIA CORPORATION is strictly prohibited.


@echo off
pushd %~dp0
SETLOCAL EnableDelayedExpansion

if "%~1"=="" goto HELP

FOR /F "tokens=* USEBACKQ" %%F IN (`helper.exe -i`) DO (
  SET STEAMVRPATH=%%F
)

if not defined STEAMVRPATH (
  echo Could not find SteamVR
  goto :eof
)

if not exist "%STEAMVRPATH%" (
  echo InstallHelper returned "%STEAMVRPATH%"
  goto :eof
)

echo.
echo Found SteamVR in "%STEAMVRPATH%"


if "%~1"=="show" goto SHOW
if "%~1"=="cleanup" goto CLEANUP

:HELP
echo drivers show: show all installed drivers
echo drivers cleanup: remove all VCR_replay drivers
goto DONE


:CLEANUP
echo.
echo removing all VCR_replay drivers
echo.
"%STEAMVRPATH%\bin\win64\vrpathreg.exe" removedriverswithname VCR_replay
goto DONE


:SHOW
echo.
echo output of 'vrpathreg show':
echo.
"%STEAMVRPATH%\bin\win64\vrpathreg.exe" show
goto DONE


:DONE
ENDLOCAL
popd