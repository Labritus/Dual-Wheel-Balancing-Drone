# People Detection Module

This module uses the Raspberry Pi camera with libcamera for people detection, sending signals to the STM32 via I2C when a person is detected.

## Features

- **Dual Detection Methods**: Uses both MobileNet SSD neural network and HOG detector for improved accuracy and resilience
- **Adaptive Image Format Handling**: Supports YUYV, MJPEG, and RGB888 formats
- **Optimized Performance**: Uses lower resolution (320x240) to improve frame rate and reduce memory usage
- **Robust Error Handling**: Comprehensive error checking and recovery mechanisms
- **Debug Logging**: Detailed logging to assist with troubleshooting
- **Automatic Test Frame**: Saves the first camera frame for quality checking

## Dependencies

- OpenCV 4.x (with DNN module)
- libcamera
- pkg-config

## Model Files

Place the following model files in the `/home/pu/Desktop/Dual-Wheel-Balancing-Drone/PeopleDetection/Model/` directory:
- `deploy.prototxt` - MobileNet SSD model configuration
- `mobilenet_iter_73000.caffemodel` - MobileNet SSD model weights
- `coco.names` - Class names file (ensure "person" is the first class)

## Building the Project

```bash
mkdir build
cd build
cmake ..
make
```

## Running the Detection Program

### Prerequisites
1. Connect the Camera to the Raspberry Pi's CSI interface
2. Enable the camera interface:
   ```bash
   sudo raspi-config
   # Select Interface Options -> Camera and enable it
   ```
3. Enable the I2C interface:
   ```bash
   sudo raspi-config
   # Select Interface Options -> I2C and enable it
   ```
4. Verify I2C connection:
   ```bash
   i2cdetect -y 1
   # Should show the STM32 device at address 0x08
   ```

### Execution

```bash
# Standard mode
./peopleDetection

# For testing without I2C hardware
./peopleDetectionTest
```

## Troubleshooting

- **Model Loading Issues**: Verify the model files exist and have the correct paths
- **Camera Not Working**: Check if the camera is enabled in raspi-config and properly connected
- **I2C Communication Failures**: Verify the I2C bus is enabled and the STM32 is responding at address 0x08
- **Performance Issues**: If detection is slow, try running `sudo rpi-update` to ensure the latest firmware

## Operation Details

1. The program initializes the camera using libcamera API
2. For each frame captured:
   - The frame is processed using MobileNet SSD to detect people
   - As a backup, HOG detection is also performed
   - When a person is detected, an I2C message "PERSON_DETECTED" is sent to the STM32
   - The frame with detection bounding boxes is displayed

## Example I2C Integration

The STM32 should be configured to receive I2C messages at address 0x08. When it receives the "PERSON_DETECTED" message, it can trigger appropriate actions (e.g., motors, alerts, etc.).

## Development Notes

- The confidence threshold is set to 0.3 (30%) - adjust in the code if needed for your environment
- A test frame is saved to `/home/pu/camera_test.jpg` on first run for debugging purposes
- The code includes multiple error handling mechanisms to prevent crashes in adverse conditions

## Extending the Project

- To add more detection classes, modify the `drawPred()` function to handle additional class IDs
- To change the I2C address or message format, modify the STM32_ADDRESS constant and sendI2CMessage() function