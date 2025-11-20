package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {
    private DcMotor right_drive;
    private DcMotor left_drive;

    public void init(HardwareMap hwMap) {
        try {
            // Try to get motors from hardware map
            right_drive = hwMap.get(DcMotor.class, "right_drive");
            left_drive = hwMap.get(DcMotor.class, "left_drive");

            // Set motor modes
            right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set motor directions
            left_drive.setDirection(DcMotor.Direction.REVERSE);
            right_drive.setDirection(DcMotor.Direction.FORWARD);

            // Set zero power behavior
            right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (IllegalArgumentException e) {
            // This catches "Device not found" errors
            throw new RuntimeException("MOTOR CONFIG ERROR: Check that motors are named 'right_drive' and 'left_drive' in Robot Configuration! Error: " + e.getMessage());
        }
    }

    public void setRight_driveSpeed(double speed) {
        if (right_drive != null) {
            right_drive.setPower(speed);
        }
    }

    public void setLeft_driveSpeed(double speed) {
        if (left_drive != null) {
            left_drive.setPower(speed);
        }
    }

    // Add method for driving with correction
    public void driveWithCorrection(double basePower, double correction) {
        double leftPower = basePower - correction;
        double rightPower = basePower + correction;

        // Clamp values to -1.0 to 1.0
        leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
        rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

        setLeft_driveSpeed(leftPower);
        setRight_driveSpeed(rightPower);
    }
}