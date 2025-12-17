package org.firstinspires.ftc.teamcode.auto_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utility_code.Stampede;

import java.util.HashMap;

@Autonomous
public class RedAuto extends OpMode {

    boolean isGoal = false;
    boolean isAudience = false;
    boolean recentIsGoalChange = false;
    boolean recentisAudienceChange = false;

    HashMap<String, double[]> drivePathAudience;
    HashMap<String, double[]> drivePathGoal;
    HashMap<String, double[]> drivePath;

    Stampede stampede;

    @Override
    public void init() {
        stampede = new Stampede();
        stampede.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {
            if (!recentIsGoalChange) {
                isGoal = true;
                isAudience = false;
                recentIsGoalChange = true;
            }
        } else {
            recentIsGoalChange = false;
        }
        if (gamepad1.dpad_down) {
            if (!recentisAudienceChange) {
                isAudience = true;
                isGoal = false;
                recentisAudienceChange = true;
            }
        } else {
            recentisAudienceChange = false;
        }
        telemetry.addData("Field position: Audience:", isAudience ? "Goal:" : isGoal);
    }

    @Override
    public void start() {
        if (isAudience) {
            drivePath = drivePathAudience;
        } else if (isGoal) {
            drivePath = drivePathGoal;
        }
        stampede.drive(0.3, 0, 0, telemetry);
        stampede.goingFor(3000);
        stampede.drive(0,0,0, telemetry);
        stampede.goingFor(100);
        stampede.drive(0,0,0.2, telemetry);
        stampede.goingFor(500);
        stampede.drive(0,0,0, telemetry);
        stampede.goingFor(100);
        stampede.drive(-0.1,0,0, telemetry);
        stampede.goingFor(2000);
        stampede.drive(0,0,0, telemetry);
        stampede.driveOther(0,0,-0.43,-0.43, telemetry);
        stampede.goingFor(1000);
        stampede.driveOther(-1,-1,-0.43,-0.43, telemetry);
        stampede.goingFor(1000);
        stampede.driveOther(0,0,0,0, telemetry);
    }

    @Override
    public void loop() {

    }
}