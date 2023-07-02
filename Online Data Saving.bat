@echo off

SET list=Desktop catkin_ws

ECHO Available folders for saving: 
(FOR %%a IN (%list%) DO (
	ECHO %%a
))

:listing
ECHO.
SET /A e=1
SET /p final="Which folder do you want to save? "
(FOR %%b IN (%list%) DO (
	IF %%b==%final% (SET /A e=0)
))
IF %e%==1 (GOTO error)

ECHO Copying the new project files from %final%
ECHO.
scp -r ubuntu@bulbabot.local:/home/ubuntu/%final% C:\Users\Alessandro\Documents\GitHub\BulbaBot2.0

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
IF %e%==1 (goto listing) ELSE (IF %e%==2 (goto resave))

:end
ECHO.
ECHO Operation completed
PAUSE