package org.firstinspires.ftc.teamcode.utility_code;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AngleTrackerIMU {

    private IMU imu;      // Control Hub IMU
    private double resetVal = 0;

    /**
     * This is the construction method
     *
     * @param imu An instance of an "imu" device.
     */
    // public AngleTrackerIMU(CenterStageBot robot) {
    public AngleTrackerIMU(IMU imu) {
            this.imu = imu;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        resetOrientation();
    }

    /**
     * Get the current orientation of the IMU in degrees.
     *
     * @return The angle in degrees.
     */
    public double getOrientation() {
        double val = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - resetVal;  //Gets angle in degrees (negatives and over 360 included)
        val = ((val % 360) + 360) % 360; //make value between 0-360 degrees

        return val;
    }

    /**
     *  This zeros the angle the robot is facing (robot centric) because we can't change the
     *  actual zero for imu (IMU decides where it thinks it zero when its turned on) so when we
     *  reset it to our zero its probably a different # for the IMU
     */
    public void resetOrientation() {
        resetVal = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * This sets the robot's angle in relation to the field's coordinates (field centric)
     * We use this to set the angle at the start in auto (:-D)
     *
     * @param heading An angle in degrees the robot is currently facing on the field.
     */
    public void setOrientation(double heading) {
        resetOrientation();
        resetVal -= heading;
    }
}
