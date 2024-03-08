:: SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
::
:: Licensed under the Apache License, Version 2.0 (the "License");
:: you may not use this file except in compliance with the License.
:: You may obtain a copy of the License at
::
:: http://www.apache.org/licenses/LICENSE-2.0
::
:: Unless required by applicable law or agreed to in writing, software
:: distributed under the License is distributed on an "AS IS" BASIS,
:: WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
:: See the License for the specific language governing permissions and
:: limitations under the License.


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