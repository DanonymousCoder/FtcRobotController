package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {
    private DcMotor right_drive;
    private DcMotor left_drive;

    public void init(HardwareMap hwMap){
        right_drive = hwMap.get(DcMotor.class, "right_drive");
        left_drive = hwMap.get(DcMotor.class, "left_drive");

        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to brake for better control
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRight_driveSpeed(double speed) {
        right_drive.setPower(speed);
    }

    public void setLeft_driveSpeed(double speed) {
        left_drive.setPower(speed);
    }

    // New method for driving with correction
    public void driveWithCorrection(double basePower, double correction) {
        // correction is added to right, subtracted from left
        // Positive correction = robot drifting left, so add power to right
        // Negative correction = robot drifting right, so add power to left

        double leftPower = basePower - correction;
        double rightPower = basePower + correction;

        // Clamp values to -1.0 to 1.0
        leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
        rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

        left_drive.setPower(leftPower);
        right_drive.setPower(rightPower);
    }
}