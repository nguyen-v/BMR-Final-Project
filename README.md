# Basics of Mobile Robotics Final Project
Implementation of global and local navigation on a Thymio. 

Basics of Mobile Robotics (MICRO-452) project at [EPFL](https://www.epfl.ch/) using the [Thymio robot](https://www.thymio.org/).

Read the report on [nbviewer](https://nbviewer.org/github/nguyen-v/BMR-Final-Project/blob/master/BMR-Final-Project.ipynb)

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

## Demonstrations

### Full demo
https://user-images.githubusercontent.com/78664993/145715880-16005536-fd62-46ec-be5c-a07e1040df7d.mp4

### Local avoidance
https://user-images.githubusercontent.com/78664993/145715899-cbdbbd22-a21b-4fe4-8f78-ea449343c4f7.mp4

### Transition between global and local navigation
https://user-images.githubusercontent.com/78664993/145715919-fb660042-d376-41aa-b545-b53bed9fa210.mp4

### Kidnapping situations
https://user-images.githubusercontent.com/78664993/145715936-ee686998-af7b-444f-903b-2372e534d5cc.mp4

### Obstructed camera
https://user-images.githubusercontent.com/78664993/145715946-d4647e57-ee34-44dd-aea6-18ee00a53104.mp4

