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
pushd %~dp0\tape

if "%~1"=="/?" goto HELP
if "%~1"=="" goto NOPARAM

:: choose here which algorithm to use
set list=%*
goto RUNFAST
::goto RUNSYNC

:: FAST: just look at run.bin and feed the next section while the current one is still playing
:: fast, but badly synchronized to the actual replay
:RUNFAST
for %%i in (%list%) do (
	call :WAITNOTRUNBIN
	echo %%i
	call runsession tracking_%%i.bin
)
goto DONE

:: SYNC: wait for replay to be ready (not replaying) 
:: then feed next section and wait for it to be busy (that it picked up the section)
:: well synchronized, but slower, also shows idle pose for a few frames because of slow sleep
:RUNSYNC
for %%i in (%list%) do (
	call :WAITREADY
	echo %%i
	call runsession tracking_%%i.bin 
	call :WAITBUSY
)
call :WAITREADY
goto DONE

:HELP
echo Replay a list of tracking files
echo No param: replay tracking_0.bin by default
echo Any number of parameters ^<p^> will play tracking_^<p^>.bin consecutively
goto :DONE

:NOPARAM
call runsession.cmd tracking_0.bin
goto DONE

:DONE
popd
goto :EOF



:: Functions to wait for events
:SLEEP
powershell -nop -c "& {Start-Sleep -m 100}"
GOTO :EOF

:: Wait for ready flag file
:WAITREADY
if not exist ready.flag (
  call :SLEEP
  GOTO :WAITREADY
)
GOTO :EOF

:: Wait for busy flag file
:WAITBUSY
if not exist busy.flag (
  call :SLEEP
  GOTO :WAITBUSY
)
GOTO :EOF

:: Wait for run.bin file to disappear
:WAITNOTRUNBIN
if exist run.bin (
  call :SLEEP
  GOTO :WAITNOTRUNBIN
)
GOTO :EOF
