package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebcamExample extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    Testbench testbench = new Testbench();

    private final double YAW_GAIN = 0.02;      // How aggressively to turn
    private final double RANGE_GAIN = 0.01;    // How aggressively to drive
    private final double DESIRED_DISTANCE = 12.0;  // Stop 12 inches away


    @Override
    public void init() {
        // Initialize your AprilTag vision system
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // Continuously update detections
        aprilTagWebcam.update();

        // Try to get a specific AprilTag (ID = 20 for example)
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        if (id20 != null) {
            telemetry.addData("Tag ID 20", "Detected");
            // Show full tag info
            telemetry.addData("Tag Info", id20.toString());
            aprilTagWebcam.displayDetectionTelemetry(id20);
        } else {
            telemetry.addData("Tag ID 20", "Not detected");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}

