@echo off

ECHO Available folders for saving: 
dir 

:listing
ECHO.
SET /A e=1
SET /p final="Which folder do you want to save? "

ECHO Uploading the new project files from %final% to Bulbabot
ECHO.
scp -r C:\Users\Alessandro\Documents\GitHub\BulbaBot2.0\%final% ubuntu@bulbabot.local:/home/ubuntu

:resave
SET /A e=0
SET /p again="Do you want to save something else? [y/n]: "
IF /I %again%==y (GOTO listing) ELSE (
	IF /I %again%==n (GOTO end) ELSE (
		SET /A e=2
		GOTO error
	)
)

:error
ECHO.
ECHO Not valid input, retry.
ECHO.
GOTO listing

:end
ECHO.
ECHO Operation completed
PAUSE