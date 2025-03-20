# People Detection Module

This module uses the Raspberry Pi camera with libcamera for people detection, sending signals to the STM32 via I2C when a person is detected.

## Dependencies

- OpenCV 4.x (with DNN module)
- libcamera
- pkg-config

## Installing libcamera on Raspberry Pi

```bash
# Install dependencies
sudo apt update
sudo apt install -y libcamera-dev libcamera-apps-lite cmake pkg-config

# If you need to install OpenCV
sudo apt install -y libopencv-dev
```

## Building the Project

```bash
mkdir build
cd build
cmake ..
make
```

## Running

Connect the Camera to the Raspberry Pi's CSI interface and make sure it's enabled:

```bash
# Check if the Camera is enabled
sudo raspi-config
# Select Interface Options -> Camera and enable it

# Run the people detection program
./peopleDetection
```

## Notes

1. The project uses an event-driven architecture, utilizing the libcamera API for camera operations, which is more efficient than using OpenCV's VideoCapture directly.
2. For people detection, it uses the MobileNet SSD model. The model files need to be placed in the same directory as the executable:
   - deploy.prototxt
   - mobilenet_iter_73000.caffemodel
   - coco.names

## I2C Connection

Make sure to properly configure the I2C device:

```bash
# Enable I2C
sudo raspi-config
# Select Interface Options -> I2C and enable it

# Check I2C devices
i2cdetect -y 1
```

The default I2C address for the STM32 is set to 0x08.
