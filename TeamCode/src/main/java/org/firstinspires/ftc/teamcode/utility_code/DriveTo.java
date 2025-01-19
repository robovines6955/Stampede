package org.firstinspires.ftc.teamcode.utility_code;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTo {
    double xTarget;
    double yTarget;
    double headingTarget;
    double targetMaxSpeed;
    boolean stopBetweenTarget;
    boolean wasStopped = true;

    /** Distance from target when robot starts slowing down (in inches) */
    //the P value of PID is 1/FULL_SPEED_DISTANCE, making FULL_SPEED_DISTANCE bigger makes the P decrease
    public static double FULL_SPEED_DISTANCE = 12;

    /** Angle from the target when the robot starts slowing down (in degrees)   */
    public static double FULL_SPEED_ANGLE = 20;

    /** Distance from target, when not stopping between actions, that it starts slowing down at (in inches) */
    public static double NO_STOP_FULL_SPEED_DISTANCE = 12;

    /** Angle from target, when not stopping between actions, that it starts slowing down at (in degrees)   */
    public static double NO_STOP_FULL_SPEED_ANGLE = 20;

    /** Distance from target where the robot is close enough and stops (in inches)  */
    public static double CLOSE_ENOUGH_DISTANCE = .25;

    /** Angle from target where the robot is close enough and stops (in degrees)    */
    public static double CLOSE_ENOUGH_ANGLE = 1;

    /** Distance from target where the robot is close enough, when not stopping between targets, and moves on to the next action (in inches)    */
    public static double NO_STOP_CLOSE_ENOUGH_DISTANCE = 1;

    /** Angle from target where the robot is close enough, when not stopping between targets, and moves on to the next action (in degrees)  */
    public static double NO_STOP_CLOSE_ENOUGH_ANGLE = 3;

    /** When stopped before starting an action, the time it takes to speed up to maxSpeedFactor (in seconds)    */
    public static double TIME_TO_SPEED_UP = .5;

    /** Factor determining how far the speed can stray from the target speed (on x and y values)   */
    public static double DISTANCE_PID_D = 0.01;

    /** Factor determining how far the speed can stray from the target speed (on degrees)   */
    public static double ANGLE_PID_D = 0.0005;

    /** Maximum amount of time until the robot gives up on the current action and moves onto the next one (in seconds) */
    public static double MAX_STALL_TIME = .25;

    /** Minimum travel distance in order to be considered moving (in inches)     */
    public static double MIN_MOVE_DIST = 0.02;

    /** Minimum angle change in order to be considering turning (in degrees)     */
    public static double MIN_MOVE_ANGLE = .04;
    double lastTimeCheck = 0;
    double lastMovementTime;
    double[] lastDistanceToDrive;
    Robot robot;
    Telemetry telemetry;
    double maxSpeedFactor = .5;

    /** To prevent stalling when going slow while not strafing (wheels have to go a certain speed in order to overcome friction)   */
    //if robot stalls then increase MIN_MOTOR_SPEED (wheels have to go a certain speed in order to overcome friction)
    public static double MIN_MOTOR_SPEED = .1;

    /** To prevent stalling when going slow while strafing (more friction when strafing so a higher minimum motor speed) */
    public static double MIN_STRAFE_SPEED = .25;
    double startTime = System.currentTimeMillis() * 0.001;
    public boolean areWeThereYet = true;

    public DriveTo(Robot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    /*
    public DriveTo(function drive(), function getTravelValues(), Telemetry telemetry) {

    }
     */

    /**
     * FOR COORDINATES: center of the field is the origin, if you are standing in the Red Alliance box looking at the field its like a normal coordinate system
     *           (Blue Alliance)
     *                    +y
     *                    |
     *   audience -x -----+--- +x not audience
     *                    |
     *                  -y
     *           (Red Alliance)
     *
     * @param x target field position in inches
     * @param y target field position in inches
     * @param heading target field centric angle in degrees
     * @param maxSpeed scale of 0 to 1 (1 being full speed)
     * @param stopBetween when false the robot doesn't have to be as accurate when arriving to target locations, allowing us to chain together paths
     */
    public void setTargetPosition(double x, double y, double heading, double maxSpeed, boolean stopBetween) {
        startTime = System.currentTimeMillis() * 0.001;
        xTarget = x;
        yTarget = y;
        headingTarget = heading;
        targetMaxSpeed = maxSpeed;
        lastTimeCheck = 0;
        areWeThereYet = false;
        stopBetweenTarget = stopBetween;
    }

    /**
     * Below is used if there is an array () versus above target position is used if 3 doubles ([0], [1], [2])
     *
     * @param position array for xTarget, yTarget, headingTarget
     */
    public void setTargetPosition(double[] position) {
        setTargetPosition(position[0], position[1], position[2]);
    }

    /**
     * Set the target position with the default speed (MaxSpeedFactor)
     *
     * @param x
     * @param y
     * @param heading
     */
    public void setTargetPosition(double x, double y, double heading) {
        setTargetPosition(x, y, heading, maxSpeedFactor);
    }

    /**
     * @param position
     * @param maxSpeed
     */
    public void setTargetPosition(double[] position, double maxSpeed) {
        setTargetPosition(position[0], position[1], position[2], maxSpeed);
    }

    /**
     * @param position
     * @param stopBetween
     */
    public void setTargetPosition(double[] position, boolean stopBetween) {
        setTargetPosition(position[0], position[1], position[2], stopBetween);
    }

    /**
     * @param x
     * @param y
     * @param heading
     * @param stopBetween
     */
    public void setTargetPosition(double x, double y, double heading, boolean stopBetween) {
        setTargetPosition(x, y, heading, maxSpeedFactor, stopBetween);
    }

    /**
     * @param position
     * @param maxSpeed
     * @param stopBetween
     */
    public void setTargetPosition(double[] position, double maxSpeed, boolean stopBetween) {
        setTargetPosition(position[0], position[1], position[2], maxSpeed, stopBetween);
    }

    /**
     * If you don't specify stopBetween true/false it defaults to true
     *
     * @param x
     * @param y
     * @param heading
     * @param maxSpeed
     */
    public void setTargetPosition(double x, double y, double heading, double maxSpeed) {
        setTargetPosition(x, y, heading, maxSpeed, true);
    }

    /**
     * Get the robot's distance to drive and reporting telemetry
     *
     * @param telemetry
     */
    public void sendTelemetry(Telemetry telemetry) {
        double[] distanceToDrive = robot.getTravelValues(xTarget, yTarget, headingTarget);
        telemetry.addData("distanceToDrive", "%4.2f %4.2f %4.2f", distanceToDrive[0], distanceToDrive[1], distanceToDrive[2]);
    }

    /**
     * get the robot's distance to drive,
     * then determine if the robot needs to stop,
     * what speed the robot needs to travel at (slow down/speed up),
     * update time,
     * and finally set the lastDistanceToDrive to the currentDistanceToDrive
     */
    public void updateDrive() {
        if (areWeThereYet) {
            return;
        }
        double[] distanceToDrive = robot.getTravelValues(xTarget, yTarget, headingTarget);
        //when robot is close enough to target distance it stops driving
        if (stopBetweenTarget && Math.abs(distanceToDrive[0]) < CLOSE_ENOUGH_DISTANCE &&
                Math.abs(distanceToDrive[1]) < CLOSE_ENOUGH_DISTANCE &&
                Math.abs(distanceToDrive[2]) < CLOSE_ENOUGH_ANGLE) {
            areWeThereYet = true;
            robot.drive(0, 0, 0, telemetry);
            wasStopped = true;
            return;
        }
        if (!stopBetweenTarget && Math.abs(distanceToDrive[0]) < NO_STOP_CLOSE_ENOUGH_DISTANCE &&
                Math.abs(distanceToDrive[1]) < NO_STOP_CLOSE_ENOUGH_DISTANCE &&
                Math.abs(distanceToDrive[2]) < NO_STOP_CLOSE_ENOUGH_ANGLE) {
            areWeThereYet = true;
            wasStopped = false;
            //no drive 0 because the robot SHOULD NOT stop
            return;
        }

        double forwardSpeed = distanceToDrive[0] / (stopBetweenTarget ? FULL_SPEED_DISTANCE : NO_STOP_FULL_SPEED_DISTANCE);
        double rightSpeed = distanceToDrive[1] / (stopBetweenTarget ? FULL_SPEED_DISTANCE : NO_STOP_FULL_SPEED_DISTANCE);
        double cwTurnSpeed = distanceToDrive[2] / (stopBetweenTarget ? FULL_SPEED_ANGLE : NO_STOP_FULL_SPEED_ANGLE);
        double currentTime = System.currentTimeMillis() * 0.001;
        //using this for getting stuck by the backdrop
        boolean didRobotMove = true;

        if (lastTimeCheck != 0) {
            double timeDelta = currentTime - lastTimeCheck;
            forwardSpeed += (distanceToDrive[0] - lastDistanceToDrive[0]) / timeDelta * DISTANCE_PID_D;
            rightSpeed += (distanceToDrive[1] - lastDistanceToDrive[1]) / timeDelta * DISTANCE_PID_D;
            cwTurnSpeed += (distanceToDrive[2] - lastDistanceToDrive[2]) / timeDelta * ANGLE_PID_D;
            didRobotMove = (
                    Math.abs(distanceToDrive[0] - lastDistanceToDrive[0]) > MIN_MOVE_DIST ||
                            Math.abs(distanceToDrive[1] - lastDistanceToDrive[1]) > MIN_MOVE_DIST ||
                            Math.abs(robot.angleDifference(distanceToDrive[2], lastDistanceToDrive[2])) > MIN_MOVE_ANGLE);
        }

        if (didRobotMove) {
            lastMovementTime = currentTime;
        } else if (currentTime - lastMovementTime > MAX_STALL_TIME) {
            //leave the loop
            areWeThereYet = true;
            wasStopped = true;
            robot.drive(0, 0, 0, telemetry);
            return;
        }
        //taking largest value (max)
        double maxAbsSpeed = Math.max(Math.max(Math.abs(forwardSpeed), Math.abs(rightSpeed)), Math.abs(cwTurnSpeed));
        double maxAllowedSpeed = targetMaxSpeed;
        if (wasStopped) {
            double speedUpSpeed = (currentTime - startTime) / TIME_TO_SPEED_UP;
            maxAllowedSpeed = Math.min(maxAllowedSpeed, speedUpSpeed);
        }

        if (Math.abs(rightSpeed) == maxAbsSpeed && maxAbsSpeed < MIN_STRAFE_SPEED) {
            //if we're going slow, and the fastest action is strafing, use the MIN_STRAFE_SPEED instead of MIN_MOTOR_SPEED
            forwardSpeed *= MIN_STRAFE_SPEED / maxAbsSpeed;
            rightSpeed *= MIN_STRAFE_SPEED / maxAbsSpeed;
            cwTurnSpeed *= MIN_STRAFE_SPEED / maxAbsSpeed;
        } else if (maxAbsSpeed > 0 && maxAbsSpeed < MIN_MOTOR_SPEED) {
            forwardSpeed *= MIN_MOTOR_SPEED / maxAbsSpeed;
            rightSpeed *= MIN_MOTOR_SPEED / maxAbsSpeed;
            cwTurnSpeed *= MIN_MOTOR_SPEED / maxAbsSpeed;
        } else if (maxAbsSpeed >= maxAllowedSpeed) {
            //scaling speed so the fastest value is 1 and the rest are proportionally lower
            forwardSpeed *= maxAllowedSpeed / maxAbsSpeed;
            rightSpeed *= maxAllowedSpeed / maxAbsSpeed;
            cwTurnSpeed *= maxAllowedSpeed / maxAbsSpeed;
        }

        lastTimeCheck = currentTime;
        lastDistanceToDrive = distanceToDrive;

        //drive
        robot.drive(forwardSpeed, rightSpeed, cwTurnSpeed, telemetry);
    }
}

