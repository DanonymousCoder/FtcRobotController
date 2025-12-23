package org.firstinspires.ftc.teamcode;  // ← NOT mechanisms!

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.CameraServo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Robot + Servo + AprilTag", group = "TeleOp")  // ← THIS MAKES IT SHOW UP!
public class AprilTagWebcamExample extends OpMode {

    TestBench bench = new TestBench();
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    CameraServo cameraServo = new CameraServo();

    private final double SERVO_SPEED = 0.015;

    @Override
    public void init() {
        try {
            bench.init(hardwareMap);
            aprilTagWebcam.init(hardwareMap, telemetry);
            cameraServo.init(hardwareMap);

            telemetry.addData("Status", "✓ ALL SYSTEMS READY");
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // DRIVE ROBOT
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double leftPower = drive + turn;
        double rightPower = drive - turn;
        bench.setLeft_driveSpeed(leftPower);
        bench.setRight_driveSpeed(rightPower);

        // SERVO CONTROL
        if (gamepad1.x) {
            cameraServo.adjustPan(-SERVO_SPEED);
        }
        if (gamepad1.b) {
            cameraServo.adjustPan(SERVO_SPEED);
        }
        if (gamepad1.a) {
            cameraServo.centerCamera();
        }

        // APRILTAG DETECTION
        aprilTagWebcam.update();
        List<AprilTagDetection> allTags = aprilTagWebcam.getDetectedTags();

        telemetry.addData("Tags Found", allTags.size());

        for (AprilTagDetection tag : allTags) {
            if (tag.ftcPose != null) {
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance", "%.1f cm", tag.ftcPose.range);
            }
        }

        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Camera Pos", "%.3f", cameraServo.getCurrentPan());
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