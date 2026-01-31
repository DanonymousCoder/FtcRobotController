package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Intake Test Auto", group = "Autonomous")
public class IntakeAuto extends LinearOpMode {

    private DcMotor intakeMotor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        // Set motor direction (change if needed)
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence
            runIntake(1.0, 2000);    // Run intake at full power for 2 seconds
            stopIntake();             // Stop
            sleep(500);               // Wait 0.5 seconds
            runIntake(-0.5, 1000);   // Reverse at half power for 1 second (outtake)
            stopIntake();             // Stop
        }
    }

    // Run intake at specified power for specified duration
    private void runIntake(double power, long milliseconds) {
        intakeMotor.setPower(power);
        sleep(milliseconds);
    }

    // Stop the intake
    private void stopIntake() {
        intakeMotor.setPower(0);
    }
}