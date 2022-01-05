# pidcontroller


SpinningDiskController has been updated to Version 2.0 using a new motor and driver as well as a larger state machine. It also has firmware based ramp inputs. 

Other Arduino sketches remain in Version 1.0.

## Installing firmware on raspberry pi via cli

### Install arduino-cli, Method 1:

Arduino CLI can be installed using:

```
wget wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_ARMv7.tar.gz
tar -xvf arduino-cli_latest_Linux_ARMv7.tar.gz
```

You should then add the download location to PATH or move arduino-cli to a location already in your PATH variable.

e.g.
```
sudo mv arduino-cli /usr/local/bin
```

### Install arduino-cli, Method 2:

Alternatively, you can use:

```
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

```
However, this appears to result in a missing library during the compiling stage and therefore an additional library needs to be installed:

```
cd .arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/lib
sudo apt install libfl2
```
Similarly, you should then add the download location to PATH or move arduino-cli to a location already in your PATH variable.

### Install SAMD board

```
arduino-cli core update-index
arduino-cli core install arduino:samd
```
Note: this may take a few minutes.

### Download Arduino script and compile

```
mkdir -p ~/sources
cd ~/sources
git clone  https://github.com/dpreid/pidcontroller.git
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
