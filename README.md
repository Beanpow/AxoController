# AxoController

This repository consist of axo controller, Moment safety detector and Info plottor.

## Overview
```
 Safety controller (to be implemented)
 ┣ Axo Controller (to be tested)
 ┃ ┗ Main thread (to be tested)
 ┃ ┗ Data recevier thread (to be tested)
 ┃ ┗ Angle detection thread (to be tested)
 ┣ Info Plotter
 ┗ Moment safety detector (to be tested)
```

## Details

### Safety controller
This is the main thread of the controller. It will be responsible for the following tasks:
- Control the robot following the desired trajectory
- Safety protection

### Axo Controller

The module is responsible for controlling the axo. It is composed of three threads: main thread, data receiver thread and angle detection thread.
* **Main thread**, which is controlling the axo following the path.
* **Data recevier thread**, which is receiving the data from the axo.
* **Angle detection thread**, which is detecting the angle of the axo, prevent the angle and current exceed the limitation.
* ~~**Info plot thread**, which is plotting the information of the axo in real time. (move to Info Plotter)~~

### Info Plotter

The module is responsible for plotting the information of the axo in real time.

### Moment safety detector

The module is responsible for communicating with the moment sensor.