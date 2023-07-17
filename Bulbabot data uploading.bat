@echo off

:start
SET /A e=0
SET /p again="Do you want to update only bulbabot pkg in catkin_ws folder? [y/n]: "
IF /I %again%==y (GOTO default) ELSE (
	IF /I %again%==n (GOTO notDefault) ELSE (
		SET /A e=1
		GOTO error
	)
)

:default
ECHO Uploading the new project files from bulbabot pkg to Bulbabot
ECHO.
scp -r C:\Users\Alessandro\Documents\GitHub\BulbaBot2.0\catkin_ws\src\bulbabot ubuntu@bulbabot.local:/home/ubuntu/catkin_ws/src
GOTO end

:notDefault
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
if %e%==1 (GOTO start) ELSE (GOTO listing)

:end
ECHO.
ECHO Operation completed
PAUSE