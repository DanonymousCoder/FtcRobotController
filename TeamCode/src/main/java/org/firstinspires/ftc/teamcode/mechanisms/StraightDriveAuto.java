package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanism.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchIMU;

@Autonomous(name="IMU Straight Drive Auto", group="Autonomous")
public class StraightDriveAuto extends LinearOpMode {

    TestBench bench = new TestBench();
    TestBenchIMU imuSensor = new TestBenchIMU();

    // PID constants for heading correction
    private final double Kp = 0.02;  // Tune this value
    private final double Ki = 0.0;
    private final double Kd = 0.0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        bench.init(hardwareMap);
        imuSensor.init(hardwareMap);

        // Calibrate IMU at start
        imuSensor.calibrateIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("IMU", "Calibrated at 0°");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run autonomous sequence
        if (opModeIsActive()) {
            // Example: Drive forward for 3 seconds
            driveStraight(1, 30.0);

            // Stop
            stopRobot();
            sleep(500);

            // Turn 90 degrees
            turnToHeading(0, 0.5, 3.0);

            // Stop
            stopRobot();
            sleep(100);

            // Drive forward again
            driveStraight(0.5, 2.0);

            // Final stop
            stopRobot();
        }

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    /**
     * Drive straight using IMU correction
     * @param power Base power (0.0 to 1.0)
     * @param timeoutSeconds How long to drive
     */
    public void driveStraight(double power, double timeoutSeconds) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeoutSeconds) {
            // Calculate heading error and correction
            double headingError = imuSensor.getHeadingError();
            double correction = headingError * Kp;

            // Apply drive with correction
            bench.driveWithCorrection(power, correction);

            // Telemetry
            telemetry.addData("Action", "Driving Straight");
            telemetry.addData("Target Heading", "%.2f°", imuSensor.getTargetHeading());
            telemetry.addData("Current Heading", "%.2f°", imuSensor.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Heading Error", "%.2f°", headingError);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Time Remaining", "%.1f s", timeoutSeconds - runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * Turn to a specific heading
     * @param targetAngle Target heading in degrees
     * @param power Turning power (0.0 to 1.0)
     * @param timeoutSeconds Maximum time to attempt turn
     */
    public void turnToHeading(double targetAngle, double power, double timeoutSeconds) {
        runtime.reset();

        // Set new target heading
        double currentHeading = imuSensor.getHeading(AngleUnit.DEGREES);
        double headingChange = targetAngle - currentHeading;

        // Normalize to -180 to 180
        while (headingChange > 180) headingChange -= 360;
        while (headingChange < -180) headingChange += 360;

        while (opModeIsActive() && runtime.seconds() < timeoutSeconds) {
            double heading = imuSensor.getHeading(AngleUnit.DEGREES);
            double error = targetAngle - heading;

            // Normalize error
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Stop if close enough (within 2 degrees)
            if (Math.abs(error) < 2.0) {
                break;
            }

            // Calculate turn power based on error
            double turnPower = error * 0.02; // Proportional control
            turnPower = Math.max(-power, Math.min(power, turnPower));

            // Apply differential steering (left and right opposite)
            bench.setLeft_driveSpeed(-turnPower);
            bench.setRight_driveSpeed(turnPower);

            // Telemetry
            telemetry.addData("Action", "Turning");
            telemetry.addData("Target Heading", "%.2f°", targetAngle);
            telemetry.addData("Current Heading", "%.2f°", heading);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Turn Power", "%.2f", turnPower);
            telemetry.update();
        }

        // Update target heading for future straight drives
        imuSensor.calibrateIMU();
    }

    /**
     * Stop all motors
     */
    public void stopRobot() {
        bench.setLeft_driveSpeed(0);
        bench.setRight_driveSpeed(0);
    }
}