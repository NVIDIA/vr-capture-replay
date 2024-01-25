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

FOR /F "tokens=* USEBACKQ" %%F IN (`.\replay\bin\win64\InstallHelper.exe -getSteamVRInstallPath`) DO (
  SET STEAMVRPATH=%%F
)

if not defined STEAMVRPATH (
  echo "Could not find SteamVR"
  goto :eof
)

if not exist "%STEAMVRPATH%" (
  echo InstallHelper returned "%STEAMVRPATH%"
  goto :eof
)

echo.
echo Found SteamVR in %STEAMVRPATH%

set "DRIVERPATH=%~dp0"
IF '%DRIVERPATH:~-1%'=='\' SET DRIVERPATH=%DRIVERPATH:~0,-1%
set "DRIVERPATH=%DRIVERPATH%\replay
echo.
echo Remove driver located in "%DRIVERPATH%"

"%STEAMVRPATH%\bin\win64\vrpathreg.exe" removedriver "%DRIVERPATH%"

echo.
echo output of 'vrpathreg show':
echo.
"%STEAMVRPATH%\bin\win64\vrpathreg.exe" show
ENDLOCAL
popd
