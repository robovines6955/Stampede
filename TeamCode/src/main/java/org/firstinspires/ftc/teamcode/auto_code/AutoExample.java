package org.firstinspires.ftc.teamcode.auto_code;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.utility_code.Robot;
import org.firstinspires.ftc.teamcode.utility_code.DriveTo;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.HashMap;

@Autonomous(name = "AUTO", group = "Autonomous")
public class AutoExample extends OpMode {
    boolean isRed = false;
    boolean isAudience = false;
    DriveTo driveTo;
    Robot robot;
    public float coords[] = new float[5];
    // This is the FIRST state for the State Machine
    String nextState = "actionStart";
    // we'll set this when we need to wait for an action to complete rather than
    // check if the lift or drive is busy.
    double wait = 0;

    //for where coords are on the field for our different auto modes (diff start positions, ect.)
    HashMap<String, double[]> drivePositionsAudienceRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsAudienceBlue = new HashMap<>();
    HashMap<String, double[]> drivePositionsBackstopRed = new HashMap<>();
    HashMap<String, double[]> drivePositionsBackstopBlue = new HashMap<>();
    HashMap<String, double[]> drivePositions;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap, true);

        driveTo = new DriveTo(robot, telemetry);

        //x, y, heading for start positions
        drivePositionsAudienceRed.put("start", new double[]{-36, -63, 90});
        drivePositionsAudienceBlue.put("start", new double[]{-36, 63, -90});
        drivePositionsBackstopRed.put("start", new double[]{12, -63, 90});
        drivePositionsBackstopBlue.put("start", new double[]{12, 63, -90});

        drivePositionsAudienceRed.put("Position 1", new double[]{-36, -40, 90});
        drivePositionsAudienceBlue.put("Position 1", new double[]{-36, 40, -90});
        drivePositionsBackstopRed.put("Position 1", new double[]{12, -40, 90});
        drivePositionsBackstopBlue.put("Position 1", new double[]{12, 40, -90});

        double ROBOT_TO_PIXEL_CENTER = 8.5;
        drivePositionsAudienceRed.put("Position 2", new double[]{-47.5 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 135});
        drivePositionsBackstopRed.put("Position 2", new double[]{0.5 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 135});
        drivePositionsAudienceBlue.put("Position 2", new double[]{-24.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -45});
        drivePositionsBackstopBlue.put("Position 2", new double[]{23.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -45});
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //Pick our hashmap for color and side
        if (isAudience && isRed) {
            drivePositions = drivePositionsAudienceRed;
        } else if (isAudience && !isRed) {
            drivePositions = drivePositionsAudienceBlue;
        } else if (!isAudience && isRed) {
            drivePositions = drivePositionsBackstopRed;
        } else {
            drivePositions = drivePositionsBackstopBlue;
        }

        //setting start position, 0 is x, 1 is y, 2 is heading
        robot.xFieldPos = drivePositions.get("start")[0];
        robot.yFieldPos = drivePositions.get("start")[1];
        robot.headingField = drivePositions.get("start")[2];
        robot.angleTracker.setOrientation(robot.headingField);
    }

    @Override
    public void loop() {

        double[] positionChange = robot.positionChange();
        robot.updateFieldPosition(positionChange[0], positionChange[1], positionChange[2]);
        telemetry.addData("Field Position (Coordinates)", "%.2f, %.2f, %.2f", robot.xFieldPos, robot.yFieldPos, robot.headingField);
        telemetry.addData("IMU Orientation", "IMU %.2f", robot.angleTracker.getOrientation());
        telemetry.addData("Next action", nextState);

        driveTo.sendTelemetry(telemetry);
        driveTo.updateDrive();

        if (!isBusy()) {
            //variable used in getDeclaredMethod cannot be changed within the loop because it is being used, so we create another variable so that we may change nextState!
            String currentState = nextState;
            try {
                // inspect our own class to see if we have an action method with the name we specified

                Method stateMethod = this.getClass().getMethod(currentState, new Class[]{});
                // invoke it with our class instance
                stateMethod.invoke(this);
                // catch exceptions to keep the compiler happy
            } catch (NoSuchMethodException exc) {
                // this will be caught when we haven't defined the method (e.g., for "done")
            } catch (IllegalAccessException exc) {
                // we don't expect this one
                telemetry.addData("exception", "IllegalAccessException when calling " + currentState + " " + exc);
            } catch (InvocationTargetException exc) {
                // we don't expect this one
                telemetry.addData("exception", "InvocationTargetException when calling " + currentState + " " +
                        exc.getTargetException() + " " + exc.getTargetException().getStackTrace()[0]);
            }
        }
    }

    @Override
    public void stop() {
        robot.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
    }

    //conditions that the robot is busy in
    Boolean isBusy() {
        //if robot isn't there yet its busy
        if (!driveTo.areWeThereYet) {
            return true;
        }
        /*
        You can check for other busy conditions like this

        if (robot.isLiftBusy()) {
            return true;
        }
        */
        if (getRuntime() < wait) {
            return true;
        }
        return false;
    }

    // This is the State Machine, it's the "steps" the robot will follow
    public void actionStart() {
        driveTo.setTargetPosition(drivePositions.get("Position 1"), 1, false);
        // Name what the next action should be
        nextState = "actionStep2";
    }

    public void actionStep2() {
        driveTo.setTargetPosition(drivePositions.get("Position 2"), .4);
        nextState = "actionStep3";
    }

    public void actionStep3() {
        driveTo.setTargetPosition(drivePositions.get("Position 1"), 1, false);
        nextState = "actionStop";
    }

    public void actionStop() {
        robot.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
        nextState = "actionDone";
    }
}