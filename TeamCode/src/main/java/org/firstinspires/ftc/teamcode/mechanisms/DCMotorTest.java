package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchIMU;

@TeleOp
public class DCMotorTest extends OpMode {
    TestBench bench = new TestBench();
    TestBenchIMU imuSensor = new TestBenchIMU();

    // PID constants for heading correction
    private final double Kp = 0.02;  // Proportional gain (tune this!)
    private final double Ki = 0.0;   // Integral gain (optional)
    private final double Kd = 0.0;   // Derivative gain (optional)

    private boolean isCalibrated = false;

    @Override
    public void init() {
        bench.init(hardwareMap);
        imuSensor.init(hardwareMap);

        telemetry.addData("Status", "Press A to calibrate IMU");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Calibrate IMU with A button (do this when robot is aligned straight)
        if (gamepad1.a && !isCalibrated) {
            imuSensor.calibrateIMU();
            isCalibrated = true;
            telemetry.addData("Status", "IMU Calibrated!");
        }

        if (!isCalibrated) {
            telemetry.addData("Status", "Press A to calibrate IMU");
            telemetry.update();
            return;
        }

        // Get desired drive power from gamepad
        double drivePower = 0.0;

        if (gamepad1.dpad_up) {
            drivePower = 0.5;  // Forward
        } else if (gamepad1.dpad_down) {
            drivePower = -0.5; // Backward
        }

        // Calculate heading error and correction
        double headingError = imuSensor.getHeadingError();
        double correction = headingError * Kp;

        // Apply drive with correction
        bench.driveWithCorrection(drivePower, correction);

        // Telemetry
        telemetry.addData("Target Heading", "%.2f°", imuSensor.getTargetHeading());
        telemetry.addData("Current Heading", "%.2f°", imuSensor.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
        telemetry.addData("Heading Error", "%.2f°", headingError);
        telemetry.addData("Correction", "%.3f", correction);
        telemetry.addData("Drive Power", "%.2f", drivePower);
        telemetry.addData("Status", "Use DPAD Up/Down to drive");
        telemetry.update();
    }
}