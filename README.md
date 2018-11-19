# AMR X communication
A project to make cros platform communication and localizaion possible.

## Relative localization and tracking using UWB

This package uses Pozyx to do relative localization of other robots. 
The package is now limited for two robots with node a frquency of 10 Hz.
Specified ranges determines wheter the node should be localizing or communicating the robots odometry.
Detailed explantion can eb found at 'Transforms' and 'Localization'.

### Dependancies

```
pip install pypozyx
pip install zlib
```

### Instructions
* Install the package on two different robots.
* Specify the Pozyx tags in (amr_x_communication/src/) 'robot_list.yaml'.
* Configure the launchfiles: one robot should have the 'do_ranging' param as 0 and another as 1. 
This is to make sure the Pozyx does stable ranging.
* Launch the nodes on both robots.

## Localization
The localization is done by interpreting the distances as radiuses of cirkles. 
The cirkles then can be used for calculating the position of where the cirkles intersect.
This intersection is the possible location of the Pozyx node. 
More information can be found [here](http://www.ambrsoft.com/TrigoCalc/Circles2/circle2intersection/CircleCircleIntersection.htm)

## Transforms
