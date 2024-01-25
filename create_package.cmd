
@echo;
@echo;
@echo [32mScript:            %~f0[0m
@echo [32mCurrent directory: %cd%[0m

@set /p VERSION=<src/aux_files/version.txt
@set VCRNAME=VCR_%VERSION%

@echo;
@echo [32mBuilding VCR version: %VERSION%[0m

@echo off
@echo;
set ZIPEXE="C:\Program Files\7-Zip\7z.exe"
if exist %ZIPEXE% (
	echo [32mFound 7zip at %ZIPEXE%[0m
) else (
	echo [31mCould not find 7zip at %ZIPEXE%[0m
	goto :END
)
@echo on

@echo;
@echo [32mLast three submits for local source:[0m
p4 changes -m3 //sw/devrel/Playpen/iesser/projects/VCR/...#have

@echo;
@echo off
for /F "tokens=2 USEBACKQ" %%F in (`p4 changes -m1 //sw/devrel/Playpen/iesser/projects/VCR/...#have`) do (
	set CHANGELIST=%%F
)
if "%VERSION:~-4%"=="_dev" (
	@echo [32mThis is an internal development build, changelist: %CHANGELIST%[0m
	set BUILD=DEV		
	set OUTDIR=%VCRNAME%_%CHANGELIST%
	set BINPACK=%OUTDIR%\%VCRNAME%_%CHANGELIST%.zip
	set PDFDOC=%OUTDIR%\%VCRNAME%_%CHANGELIST%_doc.pdf
	set HTMLDOC=%OUTDIR%\%VCRNAME%_%CHANGELIST%_html_doc.zip
	set VCRVERSION=%VERSION%_%CHANGELIST%
) else (
	@echo [32mThis is a public release build, changelist: %CHANGELIST%[0m
	set BUILD=REL
	set OUTDIR=%VCRNAME%
	set BINPACK=%OUTDIR%\%VCRNAME%.zip
	set PDFDOC=%OUTDIR%\%VCRNAME%_doc.pdf
	set HTMLDOC=%OUTDIR%\%VCRNAME%_html_doc.zip
	set VCRVERSION=%VERSION%
)
@echo on

@echo;
@echo [32mCreating VCR package for version: %VCRVERSION%[0m
@echo [32mThis script will build VCR binaries and documentation, and generate [0m
@echo [32m- Output directory:                    %OUTDIR%[0m
@echo [32m- VCR binary package:                  %BINPACK%[0m
@echo [32m- VCR documentation in HTML format:    %HTMLDOC%[0m
@echo [32m- VCR documentation in PDF format:     %PDFDOC%[0m

@echo;
@echo [32mCleaning up old build files[0m
rmdir /q /s _build
rmdir /q /s _doc
rmdir /q /s _out
rmdir /q /s %OUTDIR%

@echo;
@echo [32mBuilding VCR binaries[0m
@echo;

call build.cmd

attrib /s /d -R _out/*.*
@echo  CL %CHANGELIST% >> _out\%VCRNAME%\version.txt

@echo;
@echo [32mBuilding VCR documentation[0m
@echo;
xcopy .\doc\ .\_doc\ /E /Q

powershell -Command "(gc _doc\repo.toml) -replace 'NVVCRVERSION', '%VCRVERSION%' | Out-File -encoding ASCII _doc\repo.toml"

pushd _doc
WSL dos2unix repo
WSL dos2unix ./tools/packman/python.sh
WSL dos2unix ./tools/packman/packman
WSL ./repo docs
pushd _build\docs\vcr\latest-latex
set LATEXMKOPTS=-quiet
set WSLENV=LATEXMKOPTS
WSL make
popd
popd

@echo;
@echo [32mCreating output package[0m
@echo;
mkdir %OUTDIR%

@echo;
@echo [32mCreating %PDFDOC%[0m
copy .\_doc\_build\docs\vcr\latest-latex\vcr.pdf %PDFDOC%
copy %PDFDOC% .\_out\%VCRNAME%\.

@echo;
@echo [32mCreating %HTMLDOC%[0m
%ZIPEXE% a -r %HTMLDOC% .\_doc\_build\docs\vcr\latest\*

@echo;
@echo [32mCreating %BINPACK%[0m
%ZIPEXE% a -r %BINPACK% .\_out\*

@echo;
@echo [32mCleaning up intermediate directories[0m
@echo;

rmdir /q /s _build
rmdir /q /s _doc
rmdir /q /s _out

@echo;
@echo [32mDone.[0m
@echo;

:END
