# pidcontroller


## Active repo

This is no longer the actively developed version of this firmware. Please go [HERE](https://github.com/practable/spinner-amax/tree/main/fw) for the active version.


SpinningDiskController has been updated to Version 2.0 using a new motor and driver as well as a larger state machine. It also has firmware based ramp inputs. 

Other Arduino sketches remain in Version 1.0.

## Installing firmware on raspberry pi via cli

### Install arduino-cli:

Arduino CLI can be installed using:

```
wget wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_ARMv7.tar.gz
tar -xvf arduino-cli_latest_Linux_ARMv7.tar.gz
```

Alternatively, you can use:

```
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

```

You should then add the download location to PATH or move arduino-cli to a location already in your PATH variable.

e.g.
```
sudo mv arduino-cli /usr/local/bin
```

### Install SAMD board

```
arduino-cli core update-index
arduino-cli core install arduino:samd
```
Note: this may take a few minutes.

You may find that on compiling an arduino script an error is called due to a missing library. If so then also install the missing library:

```
cd .arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/lib
sudo apt install libfl2
```

### Download Arduino script and compile

```
mkdir -p ~/sources
cd ~/sources
sudo git clone  https://github.com/dpreid/pidcontroller.git
cd pidcontroller
arduino-cli compile --fqbn arduino:samd:nano_33_iot SpinningDiskController/ --libraries ./libraries
arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:samd:nano_33_iot SpinningDiskController/
```

### Restart data streams

```
sudo systemctl restart websocat-data
sudo systemctl restart socat-data
```

Note: for testing a fix in fork of the repo before submitting pull request, you can change the remote origin of the pidcontroller repo on the rpi

```
git remote set-url origin https://github.com/your-user-name/pidcontroller.git
```
