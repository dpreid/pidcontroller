#! /bin/sh

#Uploads a remote lab arduino script from raspberry pi to the arduino

arduino-cli compile --fqbn arduino:samd:nano_33_iot SpinningDiskController/ --libraries /home/ubuntu/arduinolibs

arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:samd:nano_33_iot SpinningDiskController/

sleep 1 

socat /dev/ttyACM0,echo=0,b57600,crnl -
