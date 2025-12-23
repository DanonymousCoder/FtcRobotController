package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.CameraServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="AprilTag Camera Control", group="TeleOp")
public class AprilTagCameraControl extends OpMode {

    // Hardware
    TestBench bench = new TestBench();
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    CameraServo cameraServo = new CameraServo();

    // Camera control settings
    private final double SERVO_SPEED = 0.015;  // How fast servo moves per button press

    // Button state tracking (for debouncing)
    private boolean lastXButton = false;
    private boolean lastBButton = false;

    @Override
    public void init() {
        try {
            // Initialize all hardware
            bench.init(hardwareMap);
            aprilTagWebcam.init(hardwareMap, telemetry);
            cameraServo.init(hardwareMap);

            telemetry.addData("Status", "✓ Initialized");
            telemetry.addData("Controls", "X=Left, B=Right");
            telemetry.addData("Camera", "Ready");
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // ========================================
        // PART 1: DRIVE ROBOT WITH LEFT STICK
        // ========================================

        double drive = -gamepad1.left_stick_y;   // Forward/backward
        double turn = gamepad1.left_stick_x;     // Left/right turning

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        bench.setLeft_driveSpeed(leftPower);
        bench.setRight_driveSpeed(rightPower);

        // ========================================
        // PART 2: MANUAL CAMERA CONTROL WITH X & B BUTTONS
        // ========================================

        // X button = Pan camera LEFT
        if (gamepad1.x && !lastXButton) {
            cameraServo.adjustPan(-SERVO_SPEED);  // Negative = left
            telemetry.addData("Camera", "← Moving Left");
        }
        lastXButton = gamepad1.x;

        // B button = Pan camera RIGHT
        if (gamepad1.b && !lastBButton) {
            cameraServo.adjustPan(SERVO_SPEED);   // Positive = right
            telemetry.addData("Camera", "Moving Right →");
        }
        lastBButton = gamepad1.b;

        // A button = Center camera
        if (gamepad1.a) {
            cameraServo.centerCamera();
            telemetry.addData("Camera", "Centered");
        }

        // ========================================
        // PART 3: DETECT ALL APRILTAGS
        // ========================================

        // Update AprilTag detections
        aprilTagWebcam.update();

        // Get ALL detected tags
        List<AprilTagDetection> allTags = aprilTagWebcam.getDetectedTags();

        // Display how many tags found
        telemetry.addData("═══════════", "APRILTAG DETECTION");
        telemetry.addData("Tags Found", allTags.size());

        // ========================================
        // PART 4: CALCULATE & DISPLAY DISTANCE/ANGLE FOR EACH TAG
        // ========================================

        if (allTags.isEmpty()) {
            telemetry.addData("Status", "❌ No tags visible");
            telemetry.addData("Tip", "Point camera at AprilTags");
        } else {
            telemetry.addData("Status", "✓ Tags detected!");

            // Loop through each detected tag
            for (AprilTagDetection tag : allTags) {
                telemetry.addLine("\n───────────────────");
                telemetry.addData("Tag ID", tag.id);

                // Check if we have position data
                if (tag.ftcPose != null) {
                    // DISTANCE (range) in centimeters
                    double distance = tag.ftcPose.range;

                    // HORIZONTAL ANGLE (bearing) in degrees
                    double horizontalAngle = tag.ftcPose.bearing;

                    // VERTICAL ANGLE (elevation) in degrees
                    double verticalAngle = tag.ftcPose.elevation;

                    // YAW (rotation of tag) in degrees
                    double yaw = tag.ftcPose.yaw;

                    // Display the data
                    telemetry.addData("  Distance", "%.1f cm (%.1f in)",
                            distance, distance / 2.54);
                    telemetry.addData("  Horizontal Angle", "%.1f°", horizontalAngle);
                    telemetry.addData("  Vertical Angle", "%.1f°", verticalAngle);
                    telemetry.addData("  Tag Rotation", "%.1f°", yaw);

                    // Position in 3D space (X, Y, Z)
                    telemetry.addData("  X (Right)", "%.1f cm", tag.ftcPose.x);
                    telemetry.addData("  Y (Forward)", "%.1f cm", tag.ftcPose.y);
                    telemetry.addData("  Z (Up)", "%.1f cm", tag.ftcPose.z);

                    // Display tag metadata if available
                    if (tag.metadata != null) {
                        telemetry.addData("  Name", tag.metadata.name);
                    }

                } else {
                    telemetry.addData("  Status", "Position data unavailable");
                }
            }
        }

        // ========================================
        // PART 5: CONTROL DISPLAY
        // ========================================

        telemetry.addData("\n═══════════", "CONTROLS");
        telemetry.addData("Left Stick", "Drive robot");
        telemetry.addData("X Button", "Pan camera LEFT");
        telemetry.addData("B Button", "Pan camera RIGHT");
        telemetry.addData("A Button", "Center camera");

        telemetry.addData("\n═══════════", "ROBOT STATUS");
        telemetry.addData("Drive Power", "%.2f", drive);
        telemetry.addData("Turn Power", "%.2f", turn);
        telemetry.addData("Camera Position", "%.3f", cameraServo.getCurrentPan());

        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean shutdown
        aprilTagWebcam.stop();
        bench.setLeft_driveSpeed(0);
        bench.setRight_driveSpeed(0);
        cameraServo.centerCamera();
    }
}