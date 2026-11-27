@echo off
rem Opens the CAN Isolator config tool in Chrome, or Edge if Chrome is not installed.
rem (The page needs Web Serial, which Chrome and Edge have and Firefox does not.)
set "PAGE=%~dp0index.html"
for %%B in ("%ProgramFiles%\Google\Chrome\Application\chrome.exe" "%ProgramFiles(x86)%\Google\Chrome\Application\chrome.exe" "%LocalAppData%\Google\Chrome\Application\chrome.exe") do (
  if exist %%B ( start "" %%B "%PAGE%" & exit /b 0 )
)
for %%B in ("%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe" "%ProgramFiles%\Microsoft\Edge\Application\msedge.exe") do (
  if exist %%B ( start "" %%B "%PAGE%" & exit /b 0 )
)
rem Neither found where expected: let Windows pick (Edge is built in).
start "" msedge "%PAGE%" || start "" "%PAGE%"
