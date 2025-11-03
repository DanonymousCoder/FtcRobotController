package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TestBenchIMU {
    private IMU imu;
    private double targetHeading = 0.0; // Stores calibrated heading

    public void init(HardwareMap hwMap){
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    // Reset IMU and set current heading as 0
    public void calibrateIMU() {
        imu.resetYaw();
        targetHeading = 0.0;
    }

    // Set the current heading as target (lock in direction)
    public void setTargetHeading() {
        targetHeading = getHeading(AngleUnit.DEGREES);
    }

    // Get the target heading
    public double getTargetHeading() {
        return targetHeading;
    }

    // Calculate heading error (how far off from target)
    public double getHeadingError() {
        double currentHeading = getHeading(AngleUnit.DEGREES);
        double error = targetHeading - currentHeading;

        // Normalize error to -180 to 180 degrees
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }
}