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
    // in the class variables
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

        drivePositionsAudienceRed.put("Prop Approach", new double[]{-36, -40, 90});
        drivePositionsAudienceBlue.put("Prop Approach", new double[]{-36, 40, -90});
        drivePositionsBackstopRed.put("Prop Approach", new double[]{12, -40, 90});
        drivePositionsBackstopBlue.put("Prop Approach", new double[]{12, 40, -90});
        double ROBOT_TO_PIXEL_CENTER = 8.5;
        drivePositionsAudienceRed.put("propLEFT", new double[]{-47.5 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 135});
        drivePositionsAudienceRed.put("propMIDDLE", new double[]{-36, -24.5 - ROBOT_TO_PIXEL_CENTER, 90});
        drivePositionsAudienceRed.put("propRIGHT", new double[]{-24.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 45});

        drivePositionsBackstopRed.put("propLEFT", new double[]{0.5 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 135});
        drivePositionsBackstopRed.put("propMIDDLE", new double[]{12, -24.5 - ROBOT_TO_PIXEL_CENTER, 90});
        drivePositionsBackstopRed.put("propRIGHT", new double[]{23.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -30 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 45});

        drivePositionsAudienceBlue.put("propLEFT", new double[]{-24.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -45});
        drivePositionsAudienceBlue.put("propMIDDLE", new double[]{-36, 24.5 + ROBOT_TO_PIXEL_CENTER, -90});
        drivePositionsAudienceBlue.put("propRIGHT", new double[]{-49 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -135});

        drivePositionsBackstopBlue.put("propLEFT", new double[]{23.5 - ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -45});
        drivePositionsBackstopBlue.put("propMIDDLE", new double[]{12, 24.5 + ROBOT_TO_PIXEL_CENTER, -90});
        drivePositionsBackstopBlue.put("propRIGHT", new double[]{-1 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), 30 + ROBOT_TO_PIXEL_CENTER / Math.sqrt(2), -135});

        drivePositionsAudienceRed.put("propApproachLEFT", new double[]{-38, -40, 135});
        drivePositionsAudienceRed.put("propApproachMIDDLE", new double[]{-36, -40, 90});
        drivePositionsAudienceRed.put("propApproachRIGHT", new double[]{-36, -40, 45});
        drivePositionsBackstopRed.put("propApproachLEFT", new double[]{12, -40, 135});
        drivePositionsBackstopRed.put("propApproachMIDDLE", new double[]{12, -40, 90});
        drivePositionsBackstopRed.put("propApproachRIGHT", new double[]{14, -40, 45});
        drivePositionsAudienceBlue.put("propApproachLEFT", new double[]{-36, 40, -45});
        drivePositionsAudienceBlue.put("propApproachMIDDLE", new double[]{-36, 40, -90});
        drivePositionsAudienceBlue.put("propApproachRIGHT", new double[]{-38, 40, -135});
        drivePositionsBackstopBlue.put("propApproachLEFT", new double[]{14, 40, -45});
        drivePositionsBackstopBlue.put("propApproachMIDDLE", new double[]{12, 40, -90});
        drivePositionsBackstopBlue.put("propApproachRIGHT", new double[]{12, 40, -135});
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

    public void actionStart() {
        driveTo.setTargetPosition(drivePositions.get("Prop Approach"), 1, false);
        nextState = "actionPlaceProp";
    }

    public void actionPlaceProp() {
        driveTo.setTargetPosition(drivePositions.get("prop"), .4);
        nextState = "actionPropAvoidPole";
    }

    public void actionPropAvoidPole() {
        driveTo.setTargetPosition(drivePositions.get("propApproach"), 1, false);
        nextState = "actionStop";
    }

    public void actionStop() {
        robot.drive(0.0, 0.0, 0.0, telemetry);
        driveTo.areWeThereYet = true;
        nextState = "actionDone";
    }
}