#!/bin/bash

sudo printf "--------------- BulbaBot - The plants guardian ---------------\n"
printf "\n	Realised by:\n"
printf "	- Alessandro Tiozzo\n"
printf "\n---- Politecnico di Torino - v2.0 ----\n"

printf "\n"
echo -n "---------- Compiling workspace"
for i in {1 2 3}
do
	echo -n "."
	sleep 0.5
done

cd
cd catkin_ws
catkin build

printf "\n\n"
echo -n "---------- Awakening BulbaBot"
for i in {1 2 3}
do
	echo -n "."
	sleep 0.5
done
printf "\n"

roslaunch bulbabot bulbabot.launch
