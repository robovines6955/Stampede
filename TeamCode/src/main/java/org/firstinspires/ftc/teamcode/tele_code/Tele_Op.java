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

import org.firstinspires.ftc.teamcode.utility_code.Stampede;

@TeleOp(name = "TeleOp")
public class Tele_Op extends OpMode {

    /* Declare OpMode members. */
    Stampede stampede;
    double x1, y1, x2;
    double outBottomSpeed, outTopSpeed, inSpeed, minSpeed;
    ElapsedTime holdTimer = new ElapsedTime();

    public void initRobot() {
        stampede = new Stampede();
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
        stampede.init(hardwareMap);
        // You can set the robot's starting orientation
        stampede.angleTracker.setOrientation(180);

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
            x2 = gamepad1.right_stick_x;
        }
        if (gamepad1.right_trigger > .4) {
            outBottomSpeed = 0.43;
            outTopSpeed = 0.43;
        } else if (!gamepad1.right_bumper) {
            outBottomSpeed = 0;
            outTopSpeed = 0;
        }
        if (gamepad1.right_bumper) {
            outBottomSpeed = 0.6;
            outTopSpeed = 0.6;
        } else if (gamepad1.right_trigger < .4) {
            outBottomSpeed = 0;
            outTopSpeed = 0;
        }

        if (gamepad1.left_trigger > .4) {
            inSpeed = 1;
        } else if (gamepad1.left_trigger < .4) {
            inSpeed = 0;
        }
        if (gamepad1.left_bumper) {
            minSpeed = 1;
        } else if (!gamepad1.left_bumper) {
            minSpeed = 0;
        }
        if (gamepad1.a) {
            stampede.limelightPositioning(telemetry);
        }
        if (gamepad1.x) {
            stampede.pusher.setPosition(0);
        } else {
            stampede.pusher.setPosition(1);
        }

        //


//Nevin Coded 10/20/20252
        //if (gamepad1.a) {
        // stampede.pushert.setPosition(0);
        //stampede.gate.setPosition(0);
        // } else {
        // stampede.pusherb.setPosition(0);
        //stampede.pushert.setPosition(0.5);
        //stampede.gate.setPosition(0.5);
//ng251022}

        x1 *= 0.75;
        y1 *= 0.75;
        x2 *= 0.75;
        stampede.drive(y1, x1, x2, telemetry);
        stampede.driveOther(-inSpeed, -minSpeed, -outBottomSpeed, -outTopSpeed, telemetry);
        telemetry.addData("Autoturning Active", corrected ? "Yes" : "No");


        stampede.updateFieldPosition();
        stampede.reportTelemetry(telemetry);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}