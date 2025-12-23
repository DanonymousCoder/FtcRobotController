package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CameraServo {
    private Servo panServo;
    private Servo tiltServo;

    // Servo position limits
    private final double PAN_MIN = 0.0;
    private final double PAN_MAX = 1.0;
    private final double PAN_CENTER = 0.5;

    private final double TILT_MIN = 0.0;
    private final double TILT_MAX = 1.0;
    private final double TILT_CENTER = 0.5;

    private double currentPan = PAN_CENTER;
    private double currentTilt = TILT_CENTER;

    // Track if servos are initialized
    private boolean panServoInitialized = false;
    private boolean tiltServoInitialized = false;

    /**
     * Initialize servo hardware with error handling
     */
    public void init(HardwareMap hwMap) {
        try {
            panServo = hwMap.get(Servo.class, "servo");
            panServoInitialized = true;
            centerCamera();
        } catch (IllegalArgumentException e) {
            panServoInitialized = false;
            throw new RuntimeException("SERVO CONFIG ERROR: Servo 'camera_pan' not found in configuration! Please add it in Robot Configuration. Error: " + e.getMessage());
        }
    }

    /**
     * Initialize with optional tilt servo
     */
    public void init(HardwareMap hwMap, boolean useTilt) {
        // Initialize pan servo
        init(hwMap);

        // Try to initialize tilt servo if requested
        if (useTilt) {
            try {
                tiltServo = hwMap.get(Servo.class, "servo");
                tiltServoInitialized = true;
            } catch (IllegalArgumentException e) {
                tiltServoInitialized = false;
                // Don't throw error for tilt - it's optional
            }
        }
    }

    /**
     * Set pan servo position (left/right)
     */
    public void setPan(double position) {
        if (!panServoInitialized || panServo == null) {
            return; // Silently fail if servo not available
        }

        position = Math.max(PAN_MIN, Math.min(PAN_MAX, position));
        currentPan = position;
        panServo.setPosition(position);
    }

    /**
     * Set tilt servo position (up/down)
     */
    public void setTilt(double position) {
        if (!tiltServoInitialized || tiltServo == null) {
            return;
        }

        position = Math.max(TILT_MIN, Math.min(TILT_MAX, position));
        currentTilt = position;
        tiltServo.setPosition(position);
    }

    /**
     * Adjust pan by delta amount
     */
    public void adjustPan(double delta) {
        setPan(currentPan + delta);
    }

    /**
     * Adjust tilt by delta amount
     */
    public void adjustTilt(double delta) {
        setTilt(currentTilt + delta);
    }

    /**
     * Center the camera
     */
    public void centerCamera() {
        setPan(PAN_CENTER);
        if (tiltServoInitialized) {
            setTilt(TILT_CENTER);
        }
    }

    /**
     * Track AprilTag with proportional control
     */
    public void trackTag(double bearing, double range, double gain) {
        if (!panServoInitialized) {
            return;
        }

        // Don't adjust if already centered (within 2 degrees)
        if (Math.abs(bearing) < 2.0) {
            return;
        }

        double panAdjustment = bearing * gain;
        adjustPan(panAdjustment);

        // Optional tilt adjustment based on range
        if (tiltServoInitialized && range > 0) {
            double tiltPosition = calculateTiltFromRange(range);
            setTilt(tiltPosition);
        }
    }

    /**
     * Calculate tilt based on distance
     */
    private double calculateTiltFromRange(double range) {
        if (range < 30.0) {
            return 0.3;  // Look down for close objects
        } else if (range < 90.0) {
            return 0.5;  // Look straight
        } else {
            return 0.6;  // Look slightly up
        }
    }

    // Getters
    public double getCurrentPan() {
        return currentPan;
    }

    public double getCurrentTilt() {
        return currentTilt;
    }

    public boolean isPanInitialized() {
        return panServoInitialized;
    }

    public boolean isTiltInitialized() {
        return tiltServoInitialized;
    }
}