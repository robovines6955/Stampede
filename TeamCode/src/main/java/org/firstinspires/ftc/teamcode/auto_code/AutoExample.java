package org.firstinspires.ftc.teamcode.auto_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility_code.DriveTo;
import org.firstinspires.ftc.teamcode.utility_code.Stampede;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;

@Autonomous(name = "AUTO", group = "Autonomous")
public class AutoExample extends OpMode {
    boolean isRed = false;
    boolean isAudience = false;
    boolean recentIsRedChange = false;
    boolean recentAudienceChange = false;
    DriveTo driveTo;
    Stampede stampede;
    // This is the FIRST state for the State Machine
    String nextState = "actionStart";
    // We'll set this when we need to wait for an action to complete rather than check if the lift or drive is busy.
    double wait = 0;

    // For where coordinates are on the field for our different auto modes (diff start positions, ect.)
    HashMap<String, double[]> drivePositionsAudienceRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsAudienceBlue = new HashMap<>();
    HashMap<String, double[]> drivePositionsBackRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsBackBlue = new HashMap<>();
    HashMap<String, double[]> drivePositions;

    @Override
    public void init() {
        stampede = new Stampede();
        stampede.init(hardwareMap);

        driveTo = new DriveTo(stampede, telemetry);

        //x, y, heading for start positions
        drivePositionsAudienceRed.put("start", new double[]{-12, -63, 90});
        drivePositionsAudienceBlue.put("start", new double[]{-12, 63, -90});
        drivePositionsBackRed.put("start", new double[]{12, -63, 90});
        drivePositionsBackBlue.put("start", new double[]{12, 63, -90});

        drivePositionsAudienceRed.put("Position 1", new double[]{-36, -40, 90});
        drivePositionsAudienceBlue.put("Position 1", new double[]{-36, 40, -90});
        drivePositionsBackRed.put("Position 1", new double[]{12, -40, 90});
        drivePositionsBackBlue.put("Position 1", new double[]{36, 40, -90});

        drivePositionsAudienceRed.put("Position 2", new double[]{24, -48, 135});
        drivePositionsAudienceBlue.put("Position 2", new double[]{-48, 60, 0});
        drivePositionsBackRed.put("Position 2", new double[]{48, -60, 0});
        drivePositionsBackBlue.put("Position 2", new double[]{-24, 48, 135 + 180});

        drivePositionsAudienceRed.put("Position 3", new double[]{48, -60, 135});
        drivePositionsAudienceBlue.put("Position 3", new double[]{-48, -48, 0});
        drivePositionsBackRed.put("Position 3", new double[]{48, -48, 0});
        drivePositionsBackBlue.put("Position 3", new double[]{-48, 60, 135 + 180});
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {        //This is our select start position
            if (!recentIsRedChange) {
                isRed = !isRed;
                recentIsRedChange = true;
            }
        } else {
            recentIsRedChange = false;
        }
        if (gamepad1.dpad_left) {      //This is our select audience position
            if (!recentAudienceChange) {
                isAudience = !isAudience;
                recentAudienceChange = true;
            }
        } else {
            recentAudienceChange = false;
        }

        telemetry.addData("Team (up)", "%s", isRed ? "RED" : "BLUE");
        telemetry.addData("Position (left)", "%s", isAudience ? "Audience" : "Back");
    }

    @Override
    public void start() {
        //Pick from our hashmap for color and side
        if (isAudience && isRed) {
            drivePositions = drivePositionsAudienceRed;
        } else if (isAudience && !isRed) {
            drivePositions = drivePositionsAudienceBlue;
        } else if (!isAudience && isRed) {
            drivePositions = drivePositionsBackRed;
        } else {
            drivePositions = drivePositionsBackBlue;
        }

        //setting start position, [0] is x, [1] is y, [2] is heading
        stampede.xFieldPos = drivePositions.get("start")[0];
        stampede.yFieldPos = drivePositions.get("start")[1];
        stampede.headingField = drivePositions.get("start")[2];
        stampede.angleTracker.setOrientation(stampede.headingField);
    }

    @Override
    public void loop() {
        stampede.updateFieldPosition();
        telemetry.addData("Field Position (Coordinates)", "%.2f, %.2f, %.2f", stampede.xFieldPos, stampede.yFieldPos, stampede.headingField);
        telemetry.addData("IMU Orientation", "IMU %.2f", stampede.angleTracker.getOrientation());
        telemetry.addData("Next action", nextState);

        driveTo.sendTelemetry(telemetry);
        driveTo.updateDrive();

        if (!isBusy()) {
            // Variable used in getDeclaredMethod cannot be changed within the loop because it is being used,
            // so we create another variable so that we may change nextState!
            String currentState = nextState;
            try {
                // Inspect our own class to see if we have an action method with the name we specified.

                Method stateMethod = this.getClass().getMethod(currentState, new Class[]{});
                // Invoke it with our class instance.
                stateMethod.invoke(this);
                // Catch exceptions to keep the compiler happy.
            } catch (NoSuchMethodException exc) {
                // This will be caught when we haven't defined the method (e.g., for "done").
            } catch (IllegalAccessException exc) {
                // We don't expect this one.
                telemetry.addData("exception", "IllegalAccessException when calling " + currentState + " " + exc);
            } catch (InvocationTargetException exc) {
                // We don't expect this one.
                telemetry.addData("exception", "InvocationTargetException when calling " + currentState + " " +
                        exc.getTargetException() + " " + exc.getTargetException().getStackTrace()[0]);
            }
        }
    }

    @Override
    public void stop() {
        stampede.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
    }

    // Conditions that the robot is busy in.
    Boolean isBusy() {
        // If robot isn't there yet its busy.
        if (!driveTo.areWeThereYet) {
            return true;
        }
        /*
        You can check for other busy conditions like this (e.g., for any other motors you want to move in auto).

        if (robot.isLiftBusy()) {
            return true;
        }
        */
        if (getRuntime() < wait) {
            return true;
        }
        return false;
    }

    // This is the State Machine, it's the "steps" the robot will follow.
    public void actionStart() {
        driveTo.setTargetPosition(drivePositions.get("Position 1"), .25);
        // This is how you can add a wait.
        //wait = getRuntime() + 5;
        // Name what the next action should be.
        nextState = "actionStep2";
    }

    public void actionStep2() {
        // stopBetween is whether the robot will stop between positions, or just drive through the position.
        driveTo.setTargetPosition(drivePositions.get("Position 2"), .25, false);
        nextState = "actionStep3";
    }

    public void actionStep3() {
        driveTo.setTargetPosition(drivePositions.get("Position 3"), .5);
        nextState = "actionStop";
    }

    public void actionStop() {
        stampede.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
        nextState = "actionDone";
    }
}