# CAM Hardware test!
This repository stores some testing for the CAMmk3 and EAGLE boards developed in 2025.

## Installation / Toolchain
This project uses platformio to build (like most of our projects). Unlike most of our projects, we use a hybrid esp-idf / arduino framework build in platformio to allow for the usage of espidf components (namely esp_h264), which requires a custom platformio build toolchain.

To be able to build this project, download platformio (either core or through the vscode marketplace), and ensure you are able to run the build command: `pio run`. This will NOT be succesful, so just ensure you are able to run it.

You will first need to download the `arduino` component by cloning the git submodule:

```bash
$ git submodule update --init --recursive
```

This will take a while, and will clone the esp32_arduino repository to `components/arduino`.

#### Build errors:

> *I am running into issues due to spaces in my path...*

Unfortunately this is a pretty debilitating issue, and is caused by your system being unable to resolve the `.platformio` directory in your user folder. A script is present in this repository (`bf_safe.py`) which will move the directory to a path-safe directory (`C:/dev/.platformio`) and symbolically link `~/.platformio` to that directory.

To be able to use this new symlinked version, you will need to set the `PLATFORMIO_CORE_DIR` environment variable to the new core location, likely `C:/dev/.platformio`. The script will give you the command to run to perform this, or you can edit the environment variable as you see fit.

> *The components/arduino folder is empty...*

You likely forgot to init the submodule, read above and run the `git submodule` command listed.

> *My flash fails even though it looks like the board flashed*

This is normal behavior and comes from a known bug in the toolchain. If you see the flashing dialog, it is likely the board flashed successfully.

