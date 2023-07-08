#!/bin/bash

sudo printf "--------------- BulbaBot - The plants guardian ---------------\n"
printf "\n	Realised by:\n"
printf "	- Alessandro Tiozzo\n"
printf "	- Alessia De Marco\n"
printf "	- Stefano Bassino\n"
printf "	- Mauro La Rocca\n"
printf "	- Francesco Gervino\n"
printf "\n---- Politecnico di Torino - Robotics 2022 - Prof. Rizzo ----\n"

printf "\n\n---------- List of available ports:\n"
printf "Port USB0:\n"
ls -l /dev/ttyUSB0

printf "\nPort USB1:\n"
ls -l /dev/ttyUSB1

printf "\nPort ACM0:\n"
ls -l /dev/ttyACM0

printf "\n"
read -p "Select the used port: " portn

printf "\n"
echo -n "---------- Giving power to Arduino"
for i in {1 2 3}
do
	echo -n "."
	sleep 0.5
done

printf "\n"
echo -n "---------- Compiling workspace"
for i in {1 2 3}
do
	echo -n "."
	sleep 0.5
done
 
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

if [ $portn = "USB0" ]; then
	roslaunch bulbabot bulbabot1.launch
else
	if [ $portn = "USB1" ]; then
		roslaunch bulbabot bulbabot2.launch 
	else
		if [ $portn = "ACM0" ]; then
			roslaunch bulbabot bulbabot0.launch
		fi
	fi
fi
