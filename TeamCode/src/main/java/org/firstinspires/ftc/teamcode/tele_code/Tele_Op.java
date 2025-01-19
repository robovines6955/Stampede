/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

package org.firstinspires.ftc.teamcode.tele_code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility_code.Robot;

@TeleOp(name = "TeleOp")
public class Tele_Op extends OpMode {

    /* Declare OpMode members. */
    Robot robot;
    double x1, y1, x2;
    double speedfactor = 0.5;
    double driveAngle = 0;
    double driveAngleCheckTime = 0;
    ElapsedTime holdTimer = new ElapsedTime();

    public void initRobot() {
        robot = new Robot();
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        initRobot();
        robot.init(hardwareMap);
        robot.angleTracker.setOrientation(180);
        //robot.configureOtos(telemetry);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        holdTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //turn correcting
        if (Math.abs(gamepad1.left_stick_y) > .2) {
            y1 = -gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.left_stick_x) > .2) {
            x1 = gamepad1.left_stick_x;
        }
        boolean corrected = false;
        if (Math.abs(gamepad1.right_stick_x) > .2) {
            // are we turning?  If so, remember our current heading
            x2 = gamepad1.right_stick_x;
            driveAngle = robot.angleTracker.getOrientation();
            driveAngleCheckTime = getRuntime() + 0.25;

        } else if (Math.abs(gamepad1.left_stick_x) > .2 || Math.abs(gamepad1.left_stick_y) > .2 &&
                getRuntime() > driveAngleCheckTime) {
            // we aren't turning, but we are moving.  Rotate back to the original heading when we started moving
            if (Math.abs(robot.angleDifference(robot.angleTracker.getOrientation(), driveAngle)) > 0.5) {
                x2 = robot.angleDifference(robot.angleTracker.getOrientation(), driveAngle) / 25;
                corrected = true;
            }
        } else {
            // we aren't moving at all, note which way we are facing
            driveAngle = robot.angleTracker.getOrientation();
        }
        if (gamepad1.left_trigger > .4) {
            speedfactor = 0.25;
        } else if (gamepad1.right_trigger > .2) {
            speedfactor = 1;
        }
        x1 *= speedfactor;
        y1 *= speedfactor;
        x2 *= speedfactor;
        robot.drive(y1, x1, x2, telemetry);

        //SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();

        double totalRotationInDeg = robot.angleTracker.getOrientation();
        telemetry.addData("Heading", "%5.2f", ((totalRotationInDeg % 360) + 360) % 360);
        telemetry.addData("corrected", corrected ? "Yes" : "No");
        //telemetry.addData("X coordinate", pos.x);
        //telemetry.addData("Y coordinate", pos.y);
        //telemetry.addData("Heading angle", pos.h);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

