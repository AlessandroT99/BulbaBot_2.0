#!/bin/bash

sudo printf "--------------- BulbaBot - The plants guardian ---------------\n"
printf "\n	Realised by:\n"
printf "	- Alessandro Tiozzo\n"
printf "\n---------------- Politecnico di Torino - v2.0 ----------------\n"

cd

sleepTime=0.3

printf "\n"
echo -n "---------- Giving root permisses to bulbabot pkg"
for i in {1 2 3}
do
	echo -n "."
	sleep $sleepTime
done
printf "\n"
sudo chmod -R a=rwx catkin_ws

cd catkin_ws

printf "\n\n"
echo -n "---------- Changing files to unix interpretation"
for i in {1 2 3}
do
	echo -n "."
	sleep $sleepTime
done
printf "\n"
dos2unix src/bulbabot/src/ControlLoop.py
dos2unix src/bulbabot/src/MainProgram.py

printf "\n"
echo -n "---------- Compiling workspace"
for i in {1 2 3}
do
	echo -n "."
	sleep $sleepTime
done
printf "\n"

#catkin build

printf "\n\n"
echo -n "---------- Awakening BulbaBot"
for i in {1 2 3}
do
	echo -n "."
	sleep $sleepTime
done
printf "\n"

roslaunch bulbabot bulbabot.launch
