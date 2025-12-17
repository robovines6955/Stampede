package org.firstinspires.ftc.teamcode.tele_code;

import static java.lang.Math.cos;
import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility_code.Stampede;

@TeleOp
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;
    Stampede stampede;

    public void initRobot() {
        stampede = new Stampede();
    }


    @Override
    public void runOpMode() throws InterruptedException
    {
        initRobot();
        telemetry.update();
        stampede.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        waitForStart();
        /*
         * Starts polling for data.
         */
        limelight.start();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("ta", result.getTa());
                    telemetry.update();
                    /*if (result.getTx() >= 0 && result.getTx() <= 0) {
                        stampede.drive(0, 0.1, 0, telemetry);
                    } else {
                        stampede.drive(0, 0, 0, telemetry);
                    }*/
                    if (result.getTx() >= 3) {
                        stampede.drive(0, 0, 0.2, telemetry);
                    } else if (result.getTx() <= -3) {
                        stampede.drive(0, 0, -0.2, telemetry);
                    } else if (result.getTx() > -3 && result.getTx() < 3) {
                        if (result.getTa() <= 0.9) {
                            stampede.drive(0.25, 0, 0, telemetry);
                        } else if (result.getTa() >= 1.1) {
                            stampede.drive(-0.25, 0, 0, telemetry);
                        } else if (result.getTa() < 1.1 && result.getTa() > 0.9) {
                            if (result.getBotpose().getPosition().x >= -0.15) {
                                stampede.drive(0, -0.5, 0, telemetry);
                            } else if (result.getBotpose().getPosition().x <= -0.3) {
                                stampede.drive(0, 0.5, 0, telemetry);
                            } else {
                                stampede.drive(0, 0, 0, telemetry);
                            }
                        }
                    }
                }
            }
        }
    }
}