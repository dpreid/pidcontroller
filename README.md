# pidcontroller


SpinningDiskController has been updated to Version 2.0 using a new motor and driver as well as a larger state machine. It also has firmware based ramp inputs. 

Other Arduino sketches remain in Version 1.0.

## Installing firmware on raspberry pi via cli


install the arduino-cli if not already installed

```
arduino-cli core update-index
arduino-cli core install arduino:samd
```


```
mkdir -p ~/sources
cd ~/sources
git clone  https://github.com/dpreid/pidcontroller.git
cd pidcontroller
arduino-cli compile --fqbn arduino:samd:nano_33_iot SpinningDiskController/ --libraries ./libraries
arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:samd:nano_33_iot SpinningDiskController/
```

Note: for testing a fix in fork of the repo before submitting pull request, you can change the remote origin of the pidcontroller repo on the rpi

```
git remote set-url origin https://github.com/your-user-name/pidcontroller.git
```
