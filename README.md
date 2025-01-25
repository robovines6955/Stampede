## Notice

The base code was originally cloned from https://github.com/FIRST-Tech-Challenge/FtcRobotController.git.  See that repository for full FTC information.

## Getting Help

The code in TeamCode was written by FTC team RoBovines 6955, if you have trouble using Stampede please contact us at team@robovines.org or create a GitHub issue.

## What is Stampede?

Stampede is a pathfinding algorithm that allows robots to drive to target positions and orientations on the field. With Stampede the robot can correct off AprilTags while in motion (unlike RoadRunner) and has a simpler calibration process. It is compatible with wheel encoders, odometry pods (three), and the optical tracking odometry sensor (otos).

## Using Stampede

To implement Stampede into your code you can either fork this repository or copy the AutoExample, Tele_Op, AngleTrackerIMU, DriveTo, and Stampede files into your code. 

<img align="left" width=348 height=400 src="https://github.com/user-attachments/assets/a71adffd-00ef-47cc-a08b-dec52f076f1a">
Stampede uses the FTC field coordinate system. Each position that you define is going to be where the center of the robot is on the field (x,y) and where the robot is facing (heading). For example, if the robot is on the field's origin facing the red alliance station, the position would be (0, 0, -90).

### Customize AngleTrackerIMU
Change the REV hub orientation to match the orientation of your control hub on the robot (if using the expansion hub IMU instead for some reason use that orientation). Keep in mind this is relative to the robot. The choices are: BACKWARD, FORWARD, UP, DOWN, LEFT, or RIGHT.
```
RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
```

### Customize Stampede
