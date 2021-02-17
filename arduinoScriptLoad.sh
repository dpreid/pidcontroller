#! /bin/sh

#Uploads a remote lab arduino script from raspberry pi to the arduino

arduino-cli compile --fqbn arduino:samd:nano_33_iot $1/

arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:samd:nano_33_iot $1/

systemctl restart websocat-data

systemctl restart socat-data
