# Instructions for Cloning
- run `git git submodule update --init --recursive` to download all libraries
- make sure to set your Arduino IDE "Sketchbook location" to be `path-to-folder/engr80-team22-2026/FinalRobotCode`
    - you can access this in the Arduino IDE Preferences

# Branches and that Jazz
Some stuff still needs to be written here. But we might need to work with git branches so that our
code has fewer merge conflicts. Let's avoid doing so for now though.

# Useful Resources
- [Oh Shit, Git!?!](https://ohshitgit.com/) is a great website for fixing issues with git
- [AS7262 Light Sensor
  Documentation](https://learn.adafruit.com/adafruit-as7262-6-channel-visible-light-sensor) and
  [Light Sensor IC
  Datasheet](https://cdn-learn.adafruit.com/assets/assets/000/052/623/original/AS7262_DS000486_2-00_%281%29.pdf?1522179774)
- [*somewhat outdated* E80 Motherboard
  Schematic](https://drive.google.com/file/d/1pk_mORYsxhedKjjqzhMrxj9H5ZVH3zqp/view)
- [Teensy 4.0 Pinout](https://github.com/KurtE/TeensyDocuments/blob/master/Teensy4%20Pins.pdf)

# Libraries
Custom E80 library files are included in the [main folder](./libraries/main/). Documentation for the default
library are in the [library README file](./libraries/main/README.md).

-----

External libraries for the GPS module, IMU, Color Sensor, and SD card interface are included as submodules.

Links to the libraries and dependencies are
- https://github.com/adafruit/Adafruit_GPS
- https://github.com/stm32duino/LSM303AGR
- https://github.com/adafruit/Adafruit_AS726x
    - https://github.com/adafruit/Adafruit_BusIO
- https://github.com/arduino-libraries/SD

# CLI
Some useful commands for the ones of us who like to use the cli (for nix-like systems only):

```ls /dev | grep usbmodem```

```arduino-cli compile --libraries=libraries --fqbn teensy:avr:teensy40 "path-to-file.ino"```

```arduino-cli upload -p /dev/tty.usbmodem"number" "arduino project name without .ino" --fqbn teensy:avr:teensy40```

```matlab -nodesktop -r "run('matlabfile.m')"```

