#! /bin/sh

#Uploads a remote lab arduino script from raspberry pi to the arduino

read -p "Enter Arduino script folder name: " folderName

arduino-cli compile --fqbn arduino:samd:nano_33_iot $folderName/

arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:samd:nano_33_iot $folderName/
