# Basics of Mobile Robotics Final Project
Implementation of global and local navigation on a Thymio. 

Basics of Mobile Robotics project at [EPFL](https://www.epfl.ch/) using the [Thymio robot](https://www.thymio.org/).

## Features
- Robust map creation and feature localisation using ArUco markers
- Global path computation with A* algorithm on grided map
- Path simplification
- Automatic port detection and connection to the Thymio
- Path following and dynamic regulation
- Thymio and objective kidnapping detection
- Map recomputation in case of kidnapping situations
- Kalman filter for navigation without measurements from the camera
- Local avoidance using potential field
## Requirements
### Python 3.x
#### External libraries
  - serial
  - numpy
  - matplotlib
  - rdp
  - opencv-contrib-python
  - Thymio

### Hardware
| Peripheral                  | Model                                                                                                    |
|-----------------------------|----------------------------------------------------------------------------------------------------------|
| Robot                       | [Thymio robot](https://www.thymio.org/)                                                                  |
| Camera                      | [Logitech Brio](https://www.thymio.org/)                                                                 | 
