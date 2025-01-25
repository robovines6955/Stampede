## Notice

The base code was originally cloned from https://github.com/FIRST-Tech-Challenge/FtcRobotController.git.  See that repository for full FTC information.

## Getting Help

The code in TeamCode was written by FTC team RoBovines 6955, if you have trouble using Stampede please contact us at team@robovines.org or create a GitHub issue.

## What is Stampede?

Stampede is a pathfinding algorithm that allows robots to drive to target positions and orientations on the field. With Stampede the robot can correct off AprilTags while in motion (unlike RoadRunner) and has a simpler calibration process. It is compatible with wheel encoders, odometry pods (three), and the optical tracking odometry sensor (otos).

## Steering Aid
In Tele_Op we added code to help the robot maintain it's heading if the driver hasn't told the robot to turn. This helps the robot drive straight and prevent it from drifting to the side. This code helps the robot needs to drive as straight as possible for the forward and strafe calibration tests.

## How to Use Stampede

To implement Stampede into your code you can either fork this repository or copy the AutoExample, Tele_Op, AngleTrackerIMU, DriveTo, and Stampede files into your code. 

<img align="left" width=348 height=400 src="https://github.com/user-attachments/assets/a71adffd-00ef-47cc-a08b-dec52f076f1a">
Stampede uses the FTC field coordinate system. Each position that you define is going to be where the center of the robot is on the field (x,y) and where the robot is facing (heading). For example, if the robot is on the field's origin facing the red alliance station, the position would be (0, 0, -90).

### Customize AngleTrackerIMU
Change the REV hub orientation to match the orientation of your control hub on the robot (if using the expansion hub IMU instead for some reason use that orientation). Keep in mind this is relative to the robot. The choices are: `BACKWARD`, `FORWARD`, `UP`, `DOWN`, `LEFT`, or `RIGHT`.
```
RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
```

### Customize Stampede
In the init section of the Stampede file, change the false to true if you are using that hardware. This example is a team that is using the IMU and wheel encoders.
```
// If using wheel encoders pass true, otherwise pass false
initWheelHardware(true);
        // If using odometry pods pass true, otherwise pass false
        if (false) {
            odopodLeft = hwMap.get(DcMotorEx.class, "odoleft");
            odopodRight = hwMap.get(DcMotorEx.class, "odoright");
            odopodMiddle = hwMap.get(DcMotorEx.class, "odomid");
        }
        // If using SparkFun otos pass true, otherwise pass false
        if (false) {
            otos = hwMap.get(SparkFunOTOS.class, "otos");
            ...more code...
        }
        // If using the IMU pass true, otherwise pass false
        if (true) {
            initTracker();
        }
```

### Customize Tele_Op
In Tele_Op the default speed is half speed and the default starting orientation is 180°. You can change these defaults in these lines.
```
double speedfactor = 0.5;
...
stampede.angleTracker.setOrientation(180);
```
Tele_Op Controls:
- **Left joystick forward/backward/left/right** drive/strafe
- **Right joystick left/right** turn
- **Hold left trigger** slow down
- **Hold right trigger** speed up

We use Logitech controllers, you may need to alter the controller-specific code depending on what controllers your team uses.

### Configure the Robot
In your driver hub name the robot's hardware. Here are the default names:
- **Wheels** fl, fr, rl, rr
- **IMU** imu
- **Odometry Pods** odoleft, odomiddle, odoright (with odomiddle being the horizontal pod)
- **OTOS** otos

Run Tele-Op and make sure when pushing the left joystick forward, the robot drives forward, left joystick left, the robot strafes left, and right joystick left, the robot turns left. Don't worry it probably won't do the correct thing first try! Change the wheel direction in `initWheelHardware` in the Stampede file (`FORWARD` or `REVERSE`) for any wheels that are spinning the wrong direction.

### General Calibration
The calibration process is similar for both wheel encoders and odometry pods (if you are using OTOS you can skip to that section), except you are going to change different variables in the Stampede file depending on whether you are using wheel encoders or odometry pods. If there are both on your robot, Stampede will use the odometry pods. 

You are going to be running three different tests for calibration, the forward test, strafe test, and turn test. You should be getting similar values in the tests that you run, discard any outliers and rerun tests if you need to. It's better to run more tests and take the average of them.

**Forward Test:** 

Drive *forward* a known distance (we did 96in) and record wheel encoder positions (use absolute value) or odo pod positions (DON'T use absolute value).

**Strafe Test:** 

Strafe *right* a known distance (we did 96in) and record wheel encoder positions (use absolute value) or odo pod positions (DON'T use absolute value).

**Turn Test:** 

Turn *clockwise* a certain number of rotations (we did 10) and record wheel encoder positions (use absolute value) or odo pod positions (DON'T use absolute value).

### Calibration (Wheel Encoders)
Run the forward test and change `FORWARD_ENCODER_COUNTS_PER_INCH` in Stampede to the values you measured. We ran one test and got 3358, 3439, 3362, and 3416 but yours will be different. Again, we ran 96in and have 4 data points so you may need to change those too if you did something different.
```
FORWARD_ENCODER_COUNTS_PER_INCH = ((3358 + 3439 + 3362 + 3416) / 4.0) / 96.0;
```
Run the strafe test and change `RIGHT_ENCODER_COUNTS_PER_INCH` to the values you measured. We ran one test and got 4109, 4056, 3899, and 3884 but yours will be different. Again, we ran 96in and have 4 data points so you may need to change those too if you did something different.
```
RIGHT_ENCODER_COUNTS_PER_INCH = ((4109 + 4056 + 3899 + 3884) / 4.0) / 96.0;
```
Run the turn test and change `CW_ENCODER_COUNTS_PER_DEGREE` to the values you measured. We ran one test and got 28859, 28843, 28849, and 28840 but yours will be different. Again, we turned 360° 10 times and have 4 data points so you may need to change those too if you did something different.
```
CW_ENCODER_COUNTS_PER_DEGREE = ((28859 + 28843 + 28849 + 28840) / 4.0) / 10.0 / 360;
```

### Calibration (Odometry Pods)
Run the forward test and change `LEFT_ENCODER_FORWARD_VALUE`, `MIDDLE_ENCODER_FORWARD_VALUE`, and `RIGHT_ENCODER_FORWARD_VALUE` to the values you measured. Each variable is the sum of that specific pod's encoder counts from each test. For example our first test we got -31297 (left value), -398 (middle value), and -32007 (right value). We then added the rest of the encoder values from the rest of our tests as we went. `FORWARD_TRAVEL` is the *total* distance the robot drove during the forward test (we drove 96in 4 times).
```
LEFT_ENCODER_FORWARD_VALUE = -31297 + -31422 + -31945 + -31571;
MIDDLE_ENCODER_FORWARD_VALUE = -398 + -840 + -669 + -686;
RIGHT_ENCODER_FORWARD_VALUE = -32007 + -32042 + -31976 + -31996;
FORWARD_TRAVEL = 96 * 4;
```

### Calibration (OTOS)
Follow the normal calibration for OTOS (directions for calibration tests are commented into `configureOtos` in the Stampede file). But don't change the values in the `configureOtos` method, change them when `configureOtos` is called in in `init`. There were the values we had on our robot for the OTOS `x`, `y`, `heading`, `angularScalar`, and `linearScalar`.
```
configureOtos(-7.125, 0, -90, 3600.0 / (3600.0 + 15.6), 96.0 / 92.6);
```
