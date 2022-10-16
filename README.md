# AxoController

This repository consist of axo controller, Moment safety detector and Info plottor.

## Overview
```
 Safety controller (to be implemented)
 ┣ Axo Controller
 ┃ ┗ Main thread
 ┃ ┗ Data recevier thread
 ┃ ┗ Angle detection thread
 ┣ Info Plotter
 ┗ Moment safety detector (to be implemented)
```

## Details

### Axo Controller

The module is responsible for controlling the axo. It is composed of three threads: main thread, data receiver thread and angle detection thread.
* **Main thread**, which is controlling the axo following the path.
* **Data recevier thread**, which is receiving the data from the axo.
* **Angle detection thread**, which is detecting the angle of the axo, prevent the angle and current exceed the limitation.
* ~~**Info plot thread**, which is plotting the information of the axo in real time. (move to Info Plotter)~~
