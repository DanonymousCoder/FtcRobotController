package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    /**
     * Initialize with HardwareMap and Telemetry
     */
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Create vision portal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        if (telemetry != null) {
            telemetry.addData("AprilTag Camera", "Initialized");
        }
    }

    /**
     * Initialize with just HardwareMap (backwards compatible)
     */
    public void init(HardwareMap hwMap) {
        init(hwMap, null);
    }

    /**
     * Update detections (call every loop)
     */
    public void update() {
        // Processor automatically updates detections
        // This method exists for future enhancements
    }

    /**
     * Get all detected AprilTags
     */
    public List<AprilTagDetection> getAllDetections() {
        if (aprilTagProcessor != null) {
            return aprilTagProcessor.getDetections();
        }
        return null;
    }

    /**
     * Get specific AprilTag by ID
     */
    public AprilTagDetection getTagBySpecificId(int id) {
        List<AprilTagDetection> detections = getAllDetections();

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == id) {
                    return detection;
                }
            }
        }

        return null;
    }

    /**
     * Display detection info on telemetry
     */
    public void displayDetectionTelemetry(AprilTagDetection detection) {
        if (telemetry == null || detection == null) return;

        telemetry.addLine("─────────────");
        telemetry.addData("Tag ID", detection.id);

        if (detection.ftcPose != null) {
            telemetry.addData("Range", "%.2f inches", detection.ftcPose.range);
            telemetry.addData("Bearing", "%.2f degrees", detection.ftcPose.bearing);
            telemetry.addData("Yaw", "%.2f degrees", detection.ftcPose.yaw);
            telemetry.addData("X", "%.2f inches", detection.ftcPose.x);
            telemetry.addData("Y", "%.2f inches", detection.ftcPose.y);
            telemetry.addData("Z", "%.2f inches", detection.ftcPose.z);
        }
    }

    /**
     * Stop the vision system
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Check if vision system is active
     */
    public boolean isActive() {
        return visionPortal != null;
    }
}