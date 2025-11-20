package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.CameraServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AprilTag Servo Tracker", group="TeleOp")
public class AprilTagServoTracker extends OpMode {

    TestBench bench = new TestBench();
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    CameraServo cameraServo = new CameraServo();

    private final int TARGET_TAG_ID = 19;
    private final double SERVO_GAIN = 0.003;

    private boolean autoTrackEnabled = true;
    private boolean lastBButtonState = false;

    @Override
    public void init() {
        try {
            // Initialize all hardware
            bench.init(hardwareMap);
            aprilTagWebcam.init(hardwareMap, telemetry);  // Now this works!
            cameraServo.init(hardwareMap);

            telemetry.addData("Status", "✓ Initialized");
            telemetry.addData("Target Tag", TARGET_TAG_ID);
            telemetry.addData("Auto Track", "Enabled");
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.addData("Cause", e.getCause());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive controls
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        bench.setLeft_driveSpeed(leftPower);
        bench.setRight_driveSpeed(rightPower);

        // Toggle tracking
        boolean currentBButton = gamepad1.b;
        if (currentBButton && !lastBButtonState) {
            autoTrackEnabled = !autoTrackEnabled;
            if (!autoTrackEnabled) {
                cameraServo.centerCamera();
            }
        }
        lastBButtonState = currentBButton;

        // Manual servo control
        if (!autoTrackEnabled) {
            double servoAdjust = gamepad1.right_stick_x * 0.01;
            cameraServo.adjustPan(servoAdjust);
        }

        // Auto tracking
        if (autoTrackEnabled) {
            aprilTagWebcam.update();
            AprilTagDetection targetTag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

            if (targetTag != null) {
                double bearing = targetTag.ftcPose.bearing;
                double range = targetTag.ftcPose.range;

                cameraServo.trackTag(bearing, range, SERVO_GAIN);

                telemetry.addData("🎯 Status", "LOCKED");
                telemetry.addData("Tag", targetTag.id);
                telemetry.addData("Bearing", "%.1f°", bearing);
                telemetry.addData("Range", "%.1f in", range);
                telemetry.addData("Servo", "%.3f", cameraServo.getCurrentPan());
            } else {
                telemetry.addData("🎯 Status", "SEARCHING");
            }
        }

        // Display
        telemetry.addData("─────", "───────");
        telemetry.addData("Mode", autoTrackEnabled ? "AUTO" : "MANUAL");
        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
        bench.setLeft_driveSpeed(0);
        bench.setRight_driveSpeed(0);
        cameraServo.centerCamera();
    }
}