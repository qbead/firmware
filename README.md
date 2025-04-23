# firmware

## Installation

### Required board files
Add the following links to Arduino IDE => Preferences => Additional boards manager URLs.

```
https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
```

### Packages
Add the following packages to Arduino IDE => Library Manager:
- Adafruit NeoPixel
- Seeed Arduino LSM6DS3

The `src` of this firmware also needs to be added as an Arduino library. This can be done two ways. The first option is to compress the `src` folder to a `.zip` file and then add it in Arduino IDE => sketch => Include library => Add .ZIP library. The second option is to create a symlink between the `src` folder and the Arduino library folder.


## Contribution guidelines

### Architecture

- The qbead class is there to serve as hardware abstraction for the motion sensor, BLE, and LEDs.

- The motion sensor is an "input device" completely uncoupled from what is shown on the LEDs output device. Various other firmware components or example sketches might connect the input and output for various purposes, but that should not be the default.

- The BLE is both for output (e.g. the measurements of the motion sensor are available over BLE) and for bridging to out output devices (e.g. setting the LEDs to a particular setting).

- Other firmware components might register their own independent BLE capabilities.

- The state class is simply a convenient way to store a block sphere and represent it in various ways. It can be used as an object to "plot" on the output device LEDs. It is not directly related to the values on the input device motion sensor (unless set to be related by an example sketch).

- There should be no blocking code and no `sleep` or `wait` functions. Rather anything that needs to be called at regular intervals should either use interrupts (rather advanced) or should keep its own page internally and just be called on each event loop iteration (similar to how the motion sensor is currently set or by having `now()-last_call < period || return` type-of short circuits).

### Code style

- Keep the codebase organized as an Arduino library: a source folder for the underlying capabilities and an example folder with sketches that can be uploaded to the device.

- Keep the formatting style and naming conventions consistent the already existing code.

- If you make a breaking change to the library, you need to update all the sketches that use the broken feature.

- We are keeping the library in a single-header form. Extreme importance is put on the legibility of the code.

- The goal should be documented in a "literal programming" fashion, so it can be read as its own story.

- As a reference from another project: Here is an [example of how we want the code to look when rendered](legibility) and here is the [source for that render](https://github.com/SpinWearables/SpinWearablesFirmware/blob/master/src/SpinWearables.h). These are from a previous project with a similar educational goal -- [the SpinWheel](https://github.com/SpinWearables/SpinWearablesFirmware)

### `git` and Pull Request workflow

- Many small PRs that build upon each other are drastically easier to review than one big PR.

- If you move code around put that change in a separate commit from the commit that edits such code -- this is necessary for the diff viewer to show the changes in an easy-to-review fashion. Otherwise the review work increases by an order of magnitude.

- After you submit your pull request, open the files tab on the pull request page and do your own informal review to make sure you have not missed something.
  
