#! /bin/sh

#Uploads a remote lab arduino script from raspberry pi to the arduino

arduino-cli compile --fqbn arduino:samd:nano_33_iot SpinningDiskController/ --libraries ./libraries

arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:samd:nano_33_iot SpinningDiskController/

systemctl restart websocat-data

systemctl restart socat-data
