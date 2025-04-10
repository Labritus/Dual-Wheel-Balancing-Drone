# People Detection Testing

This document provides information about the test program for people detection functionality without the need for I2C communication with the STM32 microcontroller.

## Overview

`peopleDetectionTest.cpp` is a simplified version of the main `peopleDetection.cpp` program that removes all I2C communication functionality while maintaining the core people detection features. This allows for isolated testing of the camera and detection algorithms without requiring a connected STM32 device.

## Key Differences from Main Program

- No I2C initialization or communication code
- Detection events are logged to the console rather than sent via I2C
- Window title changed to "People Detection Test" for clarity
- `g_personDetected` flag has been removed as it's not needed without I2C communication

## Building

```bash
mkdir build
cd build
cmake ..
make peopleDetectionTest
```

## Running the Test Program

```bash
./peopleDetectionTest
```

The program will start the camera, process frames, and display the results in a window titled "People Detection Test". When a person is detected, information about the detection will be printed to the console instead of being sent over I2C.

## Required Files

The test program still requires the following model files in the same directory:
- `deploy.prototxt`
- `mobilenet_iter_73000.caffemodel`
- `coco.names`

## Troubleshooting

If you encounter issues:

1. **Camera not found**: Make sure the camera is properly connected and enabled in raspi-config
2. **Model files not found**: Check that the required model files are in the same directory as the executable
3. **Format errors**: Verify the camera configuration parameters match your camera's capabilities

## Exiting the Program

Press Ctrl+C to send an interrupt signal and gracefully exit the program. 