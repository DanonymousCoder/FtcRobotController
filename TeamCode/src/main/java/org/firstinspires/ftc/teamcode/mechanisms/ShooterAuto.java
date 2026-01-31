package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Shooter Auto", group = "Autonomous")
public class ShooterAuto extends LinearOpMode {

    private DcMotor shooterLeft;
    private DcMotor shooterRight;

    // Fixed shoot power - adjust between 0.0 and 1.0 as needed
    static final double SHOOT_POWER = 0.8;

    // Time in ms for flywheels to spin up to full speed before launching
    static final long SPIN_UP_TIME = 1500;

    @Override
    public void runOpMode() {
        // Initialize hardware
        shooterLeft  = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        // Opposite directions so both wheels push the ball the same way
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake when stopped so wheels don't freewheel
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized - Shooter Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Step 1: Spin up flywheels
            telemetry.addData("Status", "Spinning up shooter...");
            telemetry.update();
            spinUpShooter(SHOOT_POWER);
            sleep(SPIN_UP_TIME); // Wait for wheels to reach full speed

            // Step 2: Feed ball into shooter (using intake as feeder)
            telemetry.addData("Status", "Launching ball...");
            telemetry.update();
            feedBall(1.0, 800); // Push ball into flywheels

            // Step 3: Wait for ball to clear
            sleep(500);

            // Step 4: Stop everything
            telemetry.addData("Status", "Stopping shooter");
            telemetry.update();
            stopShooter();
            stopFeeder();
        }
    }

    /**
     * Spin up both flywheel motors
     * @param power Motor power (0.0 to 1.0)
     */
    private void spinUpShooter(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    /**
     * Stop both flywheel motors
     */
    private void stopShooter() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    /**
     * Feed ball into the flywheels using intake motor as feeder
     * @param power  Feeder power (0.0 to 1.0)
     * @param millis Duration in ms
     */
    private void feedBall(double power, long millis) {
        // Change "intake" to match your intake motor name in Robot Controller
        DcMotor feeder = hardwareMap.get(DcMotor.class, "intake");
        feeder.setPower(power);
        sleep(millis);
    }

    /**
     * Stop the feeder/intake motor
     */
    private void stopFeeder() {
        DcMotor feeder = hardwareMap.get(DcMotor.class, "intake");
        feeder.setPower(0);
    }
}