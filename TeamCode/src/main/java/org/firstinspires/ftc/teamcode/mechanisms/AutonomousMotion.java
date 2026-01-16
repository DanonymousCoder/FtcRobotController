package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto: Straight & Precise", group="Competition")
public class AutonomousMotion extends LinearOpMode {

    // 1. MECHANISMS
    TestBench bench = new TestBench();
    TestBenchIMU imuSensor = new TestBenchIMU();

    // 2. CONSTANTS (TUNING)
    // TODO: Update these numbers for YOUR robot!
    // Example: GoBilda 5202 Motor (537.7) on 96mm wheels (3.78 inch)
    static final double COUNTS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = (COUNTS_PER_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // PID Gains (Twist correction)
    static final double DRIVE_GAIN = 0.05; // Strength of correction when driving straight
    static final double TURN_GAIN  = 0.02; // Strength of turning power

    @Override
    public void runOpMode() {
        // Initialize Hardware
        bench.init(hardwareMap);
        imuSensor.init(hardwareMap);

        // Ensure motors are in the correct mode for manual control + reading
        bench.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bench.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Calibrate Gyro ONCE at the start
        imuSensor.calibrateIMU();

        telemetry.addData("Status", "Ready. Calibration: " + imuSensor.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // STEP 1: Drive Forward 24 Inches
            // We aim for 0 degrees (Straight ahead)
            driveStraight(0.5, 24.0, 0.0);

            sleep(500); // Short pause to let robot settle

            // STEP 2: Turn 90 Degrees Right
            turnToHeading(-90.0);

            sleep(500);

            // STEP 3: Drive Forward 12 Inches (Still maintaining -90 heading)
            driveStraight(0.5, 12.0, -90.0);
        }
    }

    /**
     * Drive Straight using Encoder Distance + Gyro Stabilization
     * @param maxPower  Speed cap (0.0 to 1.0)
     * @param distanceInches  Distance to travel (Positive = Forward)
     * @param headingTarget  The angle we WANT to maintain (e.g., 0 for forward)
     */
    public void driveStraight(double maxPower, double distanceInches, double headingTarget) {
        // Ensure we start from a clean slate? No, we read current position.
        int moveCounts = (int)(distanceInches * COUNTS_PER_INCH);

        // Get starting positions
        int leftStart = bench.getLeftPosition();
        int rightStart = bench.getRightPosition();

        // Target is current + distance
        // We use Math.abs logic to handle forward/backward simply
        int targetLeft = leftStart + moveCounts;
        int targetRight = rightStart + moveCounts;

        // Loop until we reach the target distance
        // Logic: Keep going as long as we are NOT close enough
        // We check the AVERAGE distance traveled by both motors
        while (opModeIsActive()) {
            // 1. Where are we?
            int currentLeft = bench.getLeftPosition();
            int currentRight = bench.getRightPosition();

            // How far have we gone from start?
            double traveledLeft = Math.abs(currentLeft - leftStart);
            double traveledRight = Math.abs(currentRight - rightStart);
            double avgTraveled = (traveledLeft + traveledRight) / 2.0;

            // 2. Are we there yet?
            if (avgTraveled >= Math.abs(moveCounts)) {
                break; // Exit loop
            }

            // 3. GYRO CORRECTION
            // Calculate error: Target - Current
            double currentHeading = imuSensor.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
            double headingError = headingTarget - currentHeading;

            // Normalize error (so 359 becomes -1)
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Calculate Steer (Turn Power)
            double steer = headingError * DRIVE_GAIN;

            // 4. Set Powers
            // If distanceInches is negative, we reverse the base power
            double direction = (distanceInches < 0) ? -1.0 : 1.0;
            double leftSpeed = (maxPower * direction) - steer;
            double rightSpeed = (maxPower * direction) + steer;

            // Clip checks to make sure we don't exceed +/- 1.0
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            bench.setLeft_driveSpeed(leftSpeed);
            bench.setRight_driveSpeed(rightSpeed);

            telemetry.addData("Target", "%d", moveCounts);
            telemetry.addData("Traveled", "%.0f", avgTraveled);
            telemetry.addData("Heading Err", "%.2f", headingError);
            telemetry.update();
        }

        // Stop
        bench.setLeft_driveSpeed(0);
        bench.setRight_driveSpeed(0);
    }

    /**
     * Turn in place to a specific target heading
     * @param targetHeading Target angle in degrees
     */
    public void turnToHeading(double targetHeading) {
        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        while (opModeIsActive() && turnTimer.seconds() < 3.0) { // 3 sec timeout safety
            double currentHeading = imuSensor.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);

            double headingError = targetHeading - currentHeading;

            // Normalize error
            while (headingError > 180)  headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // SUCCESS CHECK: Are we within 1 degree?
            if (Math.abs(headingError) <= 1.0) {
                break;
            }

            // Calculate Turn Power
            double turnSpeed = headingError * TURN_GAIN;

            // Friction Feedforward: Don't let power get too low or robot won't move
            // If calculated speed is 0.05 but robot needs 0.1 to move, force it to 0.1
            double minSpeed = 0.1;
            if (Math.abs(turnSpeed) < minSpeed) {
                turnSpeed = (turnSpeed < 0) ? -minSpeed : minSpeed;
            }

            // Apply Turn (Left reverse, Right forward to turn Right)
            bench.setLeft_driveSpeed(-turnSpeed);
            bench.setRight_driveSpeed(turnSpeed);
        }

        bench.setLeft_driveSpeed(0);
        bench.setRight_driveSpeed(0);
    }
}