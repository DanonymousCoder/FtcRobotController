package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

/**
 * VISION ASSISTED TELEOP - COMPLETE SINGLE FILE
 *
 * WHAT IT DOES:
 * - Servo auto-sweeps back and forth to scan for AprilTag 19
 * - When Tag 19 is detected, servo actively tracks it
 * - Driver always has full control of robot movement
 * - Servo keeps tag in camera view even as robot moves around
 *
 * HARDWARE CONFIG REQUIREMENTS:
 * - left_drive (DC Motor)
 * - right_drive (DC Motor)
 * - camera_servo (Servo)
 * - Webcam 1 (Webcam)
 */
@TeleOp(name = "VISION ASSISTED DRIVE", group = "COMPETITION")
public class VisionAssistedTeleOp extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Servo cameraServo;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private double currentServoPosition = 0.5;
    private boolean tagDetected = false;
    private double tagDistance = 0;
    private double tagAngle = 0;
    private double tagYaw = 0;
    private final int TARGET_TAG_ID = 19;

    private boolean servoLockMode = false;
    private boolean sweepingRight = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        cameraServo = hardwareMap.get(Servo.class, "camera_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize vision
        initializeAprilTagProcessor();

        // Set initial servo position
        cameraServo.setPosition(currentServoPosition);

        telemetry.addData("Status", "Ready! Press PLAY");
        telemetry.addData("Servo Test", "Servo will AUTO-SWEEP back and forth");
        telemetry.addData("Instructions", "Watch the servo to verify it's working!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Check for AprilTags
            runVisionProcessing();

            // 2. Update servo - AUTO-SWEEP or ACTIVE TRACKING
            updateCameraServoAutoSweep();

            // 3. Drive robot - always manual
            driveRobot();

            // 4. Display info
            updateTelemetry();

            sleep(20);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * AUTO-SWEEP SERVO TEST
     * Servo automatically sweeps back and forth to verify it's working
     * If tag is detected, it actively tracks and follows it
     */
    private void updateCameraServoAutoSweep() {
        if (tagDetected) {
            // ====================================================================
            // ACTIVE TRACKING MODE: Continuously adjust to keep tag centered
            // ====================================================================
            servoLockMode = true;

            // Calculate how far off-center the tag is
            // tagYaw: negative = tag is left, positive = tag is right
            // We want to rotate the servo to bring the tag back to center

            double TRACKING_SPEED = 0.015; // How fast servo tracks (adjust this!)
            double DEADZONE = 2.0; // Don't adjust if tag is within ±2 degrees of center

            if (Math.abs(tagYaw) > DEADZONE) {
                // Tag is off-center, adjust servo to follow it
                if (tagYaw < 0) {
                    // Tag is to the LEFT of center, rotate servo LEFT to follow
                    currentServoPosition -= TRACKING_SPEED;
                } else {
                    // Tag is to the RIGHT of center, rotate servo RIGHT to follow
                    currentServoPosition += TRACKING_SPEED;
                }
            }

            // Keep servo in valid range
            currentServoPosition = Range.clip(currentServoPosition, 0.0, 1.0);

        } else {
            // ====================================================================
            // AUTO-SWEEP MODE: Continuously sweep back and forth to find tag
            // ====================================================================
            servoLockMode = false;

            // Sweep right
            if (sweepingRight) {
                currentServoPosition += 0.015; // Move right
                if (currentServoPosition >= 0.95) {
                    sweepingRight = false; // Hit right limit, reverse direction
                }
            }
            // Sweep left
            else {
                currentServoPosition -= 0.015; // Move left
                if (currentServoPosition <= 0.05) {
                    sweepingRight = true; // Hit left limit, reverse direction
                }
            }
        }

        // Apply the servo position
        cameraServo.setPosition(currentServoPosition);
    }

    /**
     * DRIVE CONTROL - ALWAYS MANUAL
     */
    private void driveRobot() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    /**
     * VISION PROCESSING
     */
    private void runVisionProcessing() {
        tagDetected = false;
        tagDistance = 0;
        tagAngle = 0;
        tagYaw = 0;

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == TARGET_TAG_ID) {
                tagDetected = true;

                if (detection.ftcPose != null) {
                    tagDistance = detection.ftcPose.range;
                    tagAngle = detection.ftcPose.bearing;
                    tagYaw = detection.ftcPose.yaw;
                }
                break;
            }
        }
    }

    /**
     * TELEMETRY
     */
    private void updateTelemetry() {
        telemetry.addLine("=== SERVO AUTO-SWEEP TEST ===");
        telemetry.addLine("");

        // Servo status with visual indicator
        telemetry.addData("Servo Mode", servoLockMode ? "🔒 ACTIVE TRACKING" : "🔄 AUTO-SWEEP");
        telemetry.addData("Servo Position", "%.2f", currentServoPosition);
        telemetry.addData("Sweep Direction", sweepingRight ? "→ RIGHT" : "← LEFT");

        // Visual position indicator
        String positionBar = createPositionBar(currentServoPosition);
        telemetry.addData("Position Bar", positionBar);

        telemetry.addLine("");

        // AprilTag status
        telemetry.addData("Looking for Tag", TARGET_TAG_ID);
        if (tagDetected) {
            telemetry.addData("Tag 19 Status", "✓ ACTIVELY TRACKING!");
            telemetry.addData("Distance", "%.1f inches", tagDistance);
            telemetry.addData("Bearing", "%.1f degrees", tagAngle);
            telemetry.addData("Yaw (off-center)", "%.1f degrees", tagYaw);

            // Show tracking status
            if (Math.abs(tagYaw) < 2.0) {
                telemetry.addData("Tracking", "🎯 CENTERED");
            } else if (tagYaw < 0) {
                telemetry.addData("Tracking", "← Adjusting LEFT");
            } else {
                telemetry.addData("Tracking", "→ Adjusting RIGHT");
            }
        } else {
            telemetry.addData("Tag 19 Status", "✗ Searching (auto-sweep active)...");
        }

        telemetry.addLine("");

        // Drive status
        telemetry.addData("Drive Mode", "MANUAL");
        telemetry.addData("Left Power", "%.2f", leftDrive.getPower());
        telemetry.addData("Right Power", "%.2f", rightDrive.getPower());

        telemetry.addLine("");
        telemetry.addData("🔧 Servo Test", "Watch servo physically move!");
        telemetry.addData("💡 If not moving", "Check wiring & config name");

        telemetry.update();
    }

    /**
     * Creates a visual bar showing servo position
     */
    private String createPositionBar(double position) {
        int barLength = 20;
        int markerPos = (int)(position * barLength);

        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < barLength; i++) {
            if (i == markerPos) {
                bar.append("█");
            } else {
                bar.append("·");
            }
        }
        bar.append("]");
        return bar.toString();
    }

    /**
     * INITIALIZE APRILTAG PROCESSOR
     */
    private void initializeAprilTagProcessor() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTagProcessor);
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);

        visionPortal = builder.build();
        visionPortal.resumeStreaming();
    }
}