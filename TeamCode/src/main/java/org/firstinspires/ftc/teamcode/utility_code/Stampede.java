package org.firstinspires.ftc.teamcode.utility_code;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Stampede {
    /* Public OpMode members. */
    public DcMotorEx driveFrontLeft = null;
    public DcMotorEx driveFrontRight = null;
    public DcMotorEx driveRearLeft = null;
    public DcMotorEx driveRearRight = null;
    public DcMotorEx odopodLeft = null;
    public DcMotorEx odopodRight = null;
    public DcMotorEx odopodMiddle = null;

    public SparkFunOTOS otos = null;

    boolean hasWheelEncoders = false;
    /**
     * FORWARD_ENCODER_COUNTS_PER_INCH, RIGHT_ENCODER_COUNTS_PER_INCH, CW_ENCODER_COUNTS_PER_DEGREE
     * are used when using wheel encoders (not odometry pods). For calibration you are going to do 3 different tests,
     * the forward test, strafe test, and turn test.
     *
     * Forward Test:
     * Drove forward 96 inches 1 time and recorded wheel encoder counts for each of the wheels (keep them all positive,
     * you might get negative encoder values but use the absolute value in the equation). The total encoder counts are
     * divided by the number of data points and the distance traveled in order to get the average encoder count per unit per wheel.
     * It's recommended to do more tests than one, here is an example if we had done 2 tests:
     * FORWARD_ENCODER_COUNTS_PER_INCH = ((3358 + 3439 + 3362 + 3416 + 3100 + 3100 + 3200 + 3200) / 8.0) / 96.0;
     */
    public static double FORWARD_ENCODER_COUNTS_PER_INCH = ((3358 + 3439 + 3362 + 3416) / 4.0) / 96.0;
    /**
     * Strafe Test:
     * Same thing as the forward except strafe right instead. Remember to use the absolute value of the wheel encoder values.
     */
    public static double RIGHT_ENCODER_COUNTS_PER_INCH = ((4109 + 4056 + 3899 + 3884) / 4.0) / 96.0;
    /**
     * Turn Test:
     * Rotated 10 times clockwise (360 degrees) for 1 test and recorded absolute value of wheel encoder counts. The total encoder counts
     * are divided by the number of data points, number of rotations, and 360 degrees.
     */
    public static double CW_ENCODER_COUNTS_PER_DEGREE = ((28859 + 28843 + 28849 + 28840) / 4.0) / 10.0 / 360;

    /**
     * The odometry calibration process is very similar to the wheel encoder calibration, except you record
     * the odometry pod encoders instead. You still do a forward test, strafe (right) test, and turn (clockwise) test. We ran 4 of each test.
     *
     * Total left odometry pod encoder count when traveling a decided forward distance (IF its negative, keep the negative sign).
     */
    static double LEFT_ENCODER_FORWARD_VALUE = -31297 + -31422 + -31945 + -31571;
    /** Total middle odometry pod encoder count when traveling a decided forward distance (IF its negative, keep the negative sign). */
    static double MIDDLE_ENCODER_FORWARD_VALUE = -398 + -840 + -669 + -686;
    /** Total right odometry pod encoder count when traveling a decided forward distance (IF its negative, keep the negative sign). */
    static double RIGHT_ENCODER_FORWARD_VALUE = -32007 + -32042 + -31976 + -31996;
    /** Decided distance from encoder forward value tests (we drove forward 96in) times number of tests (in inches). */
    static double FORWARD_TRAVEL = 96 * 4;

    /** Total left odometry pod encoder count when traveling a decided strafe distance (IF its negative, keep the negative sign). */
    static double LEFT_ENCODER_STRAFE_VALUE = 727 + 94 + -187 + 1597;
    /** Total middle odometry pod encoder count when traveling a decided strafe distance (IF its negative, keep the negative sign). */
    static double MIDDLE_ENCODER_STRAFE_VALUE = -31983 + -32027 + -32093 + -31979;
    /** Total right odometry pod encoder count when traveling a decided strafe distance (IF its negative, keep the negative sign). */
    static double RIGHT_ENCODER_STRAFE_VALUE = -455 + -1147 + -1384 + -204;
    /** Decided distance from encoder strafe value tests (we strafed right 96in) times number of tests (in inches). */
    static double STRAFE_TRAVEL = 96 * 4;

    /** Total left odometry pod encoder count when spinning a decided amount (IF its negative, keep the negative sign). */
    static double LEFT_ENCODER_CW_TURN = 132984 + 133344 + 138101 + 137789;
    /** Total middle odometry pod encoder count when spinning a decided amount (IF its negative, keep the negative sign). */
    static double MIDDLE_ENCODER_CW_TURN = 74385 + 73464 + 86852 + 86790;
    /** Total right odometry pod encoder count when spinning a decided amount (IF its negative, keep the negative sign). */
    static double RIGHT_ENCODER_CW_TURN = -126787 + -126350 + -120872 + -121079;
    /**
     * Decided amount from encoder spinning value tests [we spun clockwise (CW) ten times (3600 degrees)] times number
     * of tests (in inches).
     */
    static double CW_TURN_DEGREES = 3600 + 3600 + 3600 + 3600;

    static double[][] odomat = new double[][]{
            {LEFT_ENCODER_FORWARD_VALUE, MIDDLE_ENCODER_FORWARD_VALUE, RIGHT_ENCODER_FORWARD_VALUE},
            {LEFT_ENCODER_STRAFE_VALUE, MIDDLE_ENCODER_STRAFE_VALUE, RIGHT_ENCODER_STRAFE_VALUE},
            {LEFT_ENCODER_CW_TURN, MIDDLE_ENCODER_CW_TURN, RIGHT_ENCODER_CW_TURN}
    };
    public static double[] FORWARD_PARAMS = solve(odomat, new double[]{FORWARD_TRAVEL, 0, 0});
    public static double[] STRAFE_PARAMS = solve(odomat, new double[]{0, STRAFE_TRAVEL, 0});
    public static double[] CW_TURN_PARAMS = solve(odomat, new double[]{0, 0, CW_TURN_DEGREES});

    public AngleTrackerIMU angleTracker;

    int flLastPosition = 0;
    int frLastPosition = 0;
    int rlLastPosition = 0;
    int rrLastPosition = 0;
    int odoLeftLastPosition = 0;
    int odoRightLastPosition = 0;
    int odoMiddleLastPosition = 0;

    public double xFieldPos = 0, yFieldPos = 0, headingField = 0;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public Stampede() {

    }

    public DcMotorEx setUpEncoderMotor(
            String identifier, DcMotor.Direction direction,
            double pidf_p, double pidf_i, double pidf_d, double pidf_f) {
        return setUpEncoderMotor(identifier, direction, pidf_p, pidf_i, pidf_d, pidf_f, true);
    }

    @SuppressWarnings("SameParameterValue")
    public DcMotorEx setUpEncoderMotor(
            String identifier, DcMotor.Direction direction,
            double pidf_p, double pidf_i, double pidf_d, double pidf_f, boolean withEncoder) {
        DcMotorEx drive = hwMap.get(DcMotorEx.class, identifier);
        drive.setDirection(direction);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setZeroPowerBehavior(BRAKE);
        drive.setPower(0);
        if (!withEncoder) {
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients drivePidf = new PIDFCoefficients(pidf_p, pidf_i, pidf_d, pidf_f);
            drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, drivePidf);
        }

        return drive;
    }

    /**
     * Define and initialize wheel motors.
     *
     * @param withEncoder does your robot have wheel encoders?
     */
    public void initWheelHardware(boolean withEncoder) {
        // These PID values worked for us (using REV ultraplanetary motors)
        driveFrontLeft = setUpEncoderMotor("fl", DcMotor.Direction.FORWARD, 12, 10, 0.0, 5.0, withEncoder);
        driveFrontRight = setUpEncoderMotor("fr", DcMotor.Direction.FORWARD, 12, 10, 0.0, 5.0, withEncoder);
        driveRearLeft = setUpEncoderMotor("rl", DcMotor.Direction.REVERSE, 12, 10, 0.0, 5.0, withEncoder);
        driveRearRight = setUpEncoderMotor("rr", DcMotor.Direction.REVERSE, 12, 10, 0.0, 5.0, withEncoder);

        hasWheelEncoders = withEncoder;
    }

    /**
     * Initializes the angle tracker (IMU).
     */
    public void initTracker() {
        angleTracker = new AngleTrackerIMU(hwMap.get(IMU.class, "imu"));
    }

    /**
     * Initialize standard Hardware interfaces.
     *
     * @param ahwMap Hardware map from opmode
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

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
            // put otos calibration values here
            configureOtos(-7.125, 0, -90, 3600.0 / (3600.0 + 15.6), 96.0 / 92.6);
            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(xFieldPos, yFieldPos, headingField);
            otos.setPosition(currentPosition);
        }
        // You can use the REV controller's IMU for better orientation
        // If using the IMU pass true, otherwise pass false
        if (true) {
            initTracker();
        }
    }

    /**
     * Calculate speed for the wheels, sets the power for the wheels, and sending telemetry.
     * This is for mechanum robots.
     *
     * @param forward     speed forward [-1, 1]
     * @param strafeRight speed right [-1, 1]
     * @param turnCW      speed turn [-1, 1]
     * @param telemetry
     */
    public void drive(double forward, double strafeRight, double turnCW, Telemetry telemetry) {

        double speedfr = forward - strafeRight - turnCW;
        double speedfl = forward + strafeRight + turnCW;
        double speedrl = forward - strafeRight + turnCW;
        double speedrr = forward + strafeRight - turnCW;

        double max = Math.max(Math.max(Math.abs(speedfl), Math.abs(speedfr)), Math.max(Math.abs(speedrl), Math.abs(speedrr)));

        if (max > 1) { // Use abs value so we don't divide a neg by a neg and have it go forward unexpectedly
            speedfl /= max;
            speedfr /= max;
            speedrl /= max;
            speedrr /= max;
        }

        driveFrontLeft.setPower(speedfl);
        driveRearLeft.setPower(speedrl);
        driveFrontRight.setPower(speedfr);
        driveRearRight.setPower(speedrr);
    }

    /**
     * Update the robot's position since the last check.
     */
    public void updateFieldPosition() {
        double forwardDistance = 0;
        double rightDistance = 0;
        double cwTurnAngle = 0;

        // if we aren't using odopos or otos, we must use wheel encoders
        if (odopodLeft == null && otos == null) {
            int flPosition = driveFrontLeft.getCurrentPosition();
            int frPosition = driveFrontRight.getCurrentPosition();
            int rlPosition = driveRearLeft.getCurrentPosition();
            int rrPosition = driveRearRight.getCurrentPosition();

            // how much did we move since we last asked
            int flPositionChange = flPosition - flLastPosition;
            int frPositionChange = frPosition - frLastPosition;
            int rlPositionChange = rlPosition - rlLastPosition;
            int rrPositionChange = rrPosition - rrLastPosition;

            // how far has robot moved, how many inches and degrees changed (forward, strafe, turn)
            // inches traveled forward
            forwardDistance = ((flPositionChange - frPositionChange - rlPositionChange + rrPositionChange) / 4.0) / FORWARD_ENCODER_COUNTS_PER_INCH;
            // inches traveled right
            rightDistance = ((flPositionChange + frPositionChange + rlPositionChange + rrPositionChange) / 4.0) / RIGHT_ENCODER_COUNTS_PER_INCH;
            // turn angle in degrees
            cwTurnAngle = ((flPositionChange + frPositionChange - rlPositionChange - rrPositionChange) / 4.0) / CW_ENCODER_COUNTS_PER_DEGREE;

            // updating new last position
            flLastPosition = flPosition;
            frLastPosition = frPosition;
            rlLastPosition = rlPosition;
            rrLastPosition = rrPosition;
        } else if (odopodLeft != null) {
            int rightPosition = odopodRight.getCurrentPosition();
            int leftPosition = odopodLeft.getCurrentPosition();
            int midPosition = odopodMiddle.getCurrentPosition();

            // how much did the robot move since we last asked
            int rightPod = rightPosition - odoRightLastPosition;
            int leftPod = leftPosition - odoLeftLastPosition;
            int midPod = midPosition - odoMiddleLastPosition;

            // how far has robot moved, how many inches and degrees changed (forward, strafe, turn)
            // turn angle in degrees
            //double cwTurnAngle = ((flPositionChange + frPositionChange - rlPositionChange - rrPositionChange) / 4.0) / CW_ENCODER_COUNTS_PER_DEGREE;
            //inches traveled forward
            //double forwardDistance = ((flPositionChange - frPositionChange - rlPositionChange + rrPositionChange) / 4.0) / FORWARD_ENCODER_COUNTS_PER_INCH;
            //inches traveled right
            //double rightDistance = ((flPositionChange + frPositionChange + rlPositionChange + rrPositionChange) / 4.0) / RIGHT_ENCODER_COUNTS_PER_INCH;
            forwardDistance = FORWARD_PARAMS[0] * leftPod + FORWARD_PARAMS[1] * midPod + FORWARD_PARAMS[2] * rightPod;
            rightDistance = STRAFE_PARAMS[0] * leftPod + STRAFE_PARAMS[1] * midPod + STRAFE_PARAMS[2] * rightPod;
            cwTurnAngle = CW_TURN_PARAMS[0] * leftPod + CW_TURN_PARAMS[1] * midPod + CW_TURN_PARAMS[2] * rightPod;

            odoRightLastPosition = rightPosition;
            odoLeftLastPosition = leftPosition;
            odoMiddleLastPosition = midPosition;
        } else {
            // if we don't have wheel encoders or odopods, we must have an otos
            SparkFunOTOS.Pose2D currPos = otos.getPosition();
            xFieldPos = currPos.x;
            yFieldPos = currPos.y;
            headingField = currPos.h;
        }
        // apply the position change from wheel encoders or odopods
        if (forwardDistance != 0.0 || rightDistance != 0.0 || cwTurnAngle != 0.0) {
            // same as double averageHeading = (headingField + (headingField - cwTurn)) / 2;
            double avgHeading = headingField - cwTurnAngle / 2;
            headingField -= cwTurnAngle;
            // effect of forward movement
            xFieldPos += forwardDistance * Math.cos(Math.toRadians(avgHeading));
            yFieldPos += forwardDistance * Math.sin(Math.toRadians(avgHeading));
            //effect of right movement
            xFieldPos += rightDistance * Math.sin(Math.toRadians(avgHeading));
            yFieldPos -= rightDistance * Math.cos(Math.toRadians(avgHeading));
        }

        // if you are using the imu, it overrules orientation from other methods
        if (angleTracker != null) {
            // this is where the IMU thinks heading is
            headingField = angleTracker.getOrientation();
        }
    }

    /**
     * Compute the distance the robot needs to travel from the current field position to a target
     * position.
     *
     * @param xTargetPos    Target position in field coordinates (in inches)
     * @param yTargetPos    Target position in field coordinates (in inches)
     * @param headingTarget Target heading in field coordinates (in degrees)
     * @return Distances to travel forward (in inches), right (in inches), and turn clockwise (in degrees)
     */
    public double[] getTravelValues(double xTargetPos, double yTargetPos, double headingTarget) {
        double deltaX = xTargetPos - xFieldPos;
        double deltaY = yTargetPos - yFieldPos;
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double fieldBearing = Math.atan2(deltaY, deltaX);
        double robotBearing = fieldBearing - Math.toRadians(headingField);
        double forwardTravel = Math.cos(robotBearing) * distance;
        double leftTravel = Math.sin(robotBearing) * distance;
        //if rightTravel is positive go right and if leftTravel is positive go left
        double rightTravel = -leftTravel;
        double cwTurnAngle = headingField - headingTarget;
        //to get angle between 0-360
        cwTurnAngle = ((cwTurnAngle % 360) + 360) % 360;
        //to turn the shortest possible distance
        if (cwTurnAngle > 180) {
            cwTurnAngle -= 360;
        }
        return new double[]{forwardTravel, rightTravel, cwTurnAngle};

    }

    /**
     * @param angle1 (in degrees)
     * @param angle2 (in degrees)
     * @return angle 1 - angle 2 out of 360 (in degrees)
     */
    public double angleDifference(double angle1, double angle2) {
        double angleDiff = angle1 - angle2;
        //to get angle between 0-360
        angleDiff = ((angleDiff % 360) + 360) % 360;
        //to turn the shortest possible distance
        if (angleDiff > 180) {
            angleDiff -= 360;
        }
        return angleDiff;
    }

    /**
     * Sets the robot's field position to a known set of coordinates.
     *
     * @param x Field position (in inches)
     * @param y Field position (in inches)
     * @param heading Field orientation (in degrees)
     */
    public void setFieldPosition(double x, double y, double heading) {
        xFieldPos = x;
        yFieldPos = y;
        headingField = heading;
        if (angleTracker != null) {
            angleTracker.setOrientation(headingField);
        }
        if (otos != null) {
            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(xFieldPos, yFieldPos, headingField);
            otos.setPosition(currentPosition);
        }
    }

    /** @return maximum drive speed out of all the motors */
    public double currentAbsDriveSpeed() {
        return Math.max(
                Math.max(Math.abs(driveFrontLeft.getPower()), Math.abs(driveFrontRight.getPower())),
                Math.max(Math.abs(driveRearLeft.getPower()), Math.abs(driveRearRight.getPower()))
        );
    }

    public void configureOtos(double x, double y, double h, double angularScalar, double linearScalar) {
        // Set the desired units for linear and angular measurements
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        //SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-6.9, -.9, 180);
        //Consider y axis being 0 we think.
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(x, y, h);
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otos.setAngularScalar(angularScalar);
        otos.setLinearScalar(linearScalar);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
    }

    /**
     * Log information that you will need for calibration in Tele_Op.
     *
     * @param telemetry
     */
    public void reportTelemetry(Telemetry telemetry){
        if(hasWheelEncoders){
            telemetry.addData("Wheel Encoder Postions", "fl: %d, fr: %d, rl: %d, rr: %d",
                    driveFrontLeft.getCurrentPosition(), driveFrontRight.getCurrentPosition(),
                    driveRearLeft.getCurrentPosition(), driveRearRight.getCurrentPosition());
        }
        if(odopodLeft != null){
            telemetry.addData("Odo Pod Postions", "left: %d, middle: %d, right: %d",
                    odopodLeft.getCurrentPosition(), odopodMiddle.getCurrentPosition(),
                    odopodRight.getCurrentPosition());
        }
        if(otos != null){
            SparkFunOTOS.Pose2D pos = otos.getPosition();
            telemetry.addData("OTOS Postions", "x: %4.2f, y: %4.2f, heading: %4.2f",
                    pos.x, pos.y, pos.h);
        }
        if (angleTracker != null) {
            double totalRotationInDeg = angleTracker.getOrientation();
            telemetry.addData("Heading", "%5.2f", ((totalRotationInDeg % 360) + 360) % 360);
        }
        telemetry.addData("Field Postion", "x: %4.2f, y: %4.2f, heading: %4.2f",
                xFieldPos, yFieldPos, headingField);
    }
    /**
     * Solves a system of three linear equations with three unknowns.
     * The function takes a 3x3 coefficient matrix A and a 3-element vector b,
     * and returns the solution as a 3-element array.
     * This implementation is based on the Cramer's rule algorithm.
     *
     * This removes the Apache library.
     *
     * @param A the 3x3 coefficient matrix
     * @param b the 3-element vector of constants
     * @return the solution as a 3-element array
     */
    public static double[] solve(double[][] A, double[] b) {
        double determinant = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) +
                A[0][1] * (A[1][2] * A[2][0] - A[1][0] * A[2][2]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        if (Math.abs(determinant) < 1e-10) {
            throw new IllegalArgumentException("The system of equations is ill conditioned.");
        }
        double x = (b[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) +
                A[0][1] * (A[1][2] * b[2] - b[1] * A[2][2]) +
                A[0][2] * (b[1] * A[2][1] - A[1][1] * b[2])) / determinant;
        double y = (A[0][0] * (b[1] * A[2][2] - A[1][2] * b[2]) +
                b[0] * (A[1][2] * A[2][0] - A[1][0] * A[2][2]) +
                A[0][2] * (A[1][0] * b[2] - b[1] * A[2][0])) / determinant;
        double z = (A[0][0] * (A[1][1] * b[2] - b[1] * A[2][1]) +
                A[0][1] * (b[1] * A[2][0] - A[1][0] * b[2]) +
                b[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) / determinant;
        return new double[]{x, y, z};
    }
    }
