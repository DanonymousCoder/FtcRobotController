package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CameraServo {
    private Servo panServo;    // Left/Right rotation
    private Servo tiltServo;   // Up/Down rotation (optional)

    // Servo position limits (adjust based on your servo's range)
    private final double PAN_MIN = 0.0;    // Leftmost position
    private final double PAN_MAX = 1.0;    // Rightmost position
    private final double PAN_CENTER = 0.5; // Center position

    private final double TILT_MIN = 0.0;   // Down position
    private final double TILT_MAX = 1.0;   // Up position
    private final double TILT_CENTER = 0.5;

    // Current servo positions
    private double currentPan = PAN_CENTER;
    private double currentTilt = TILT_CENTER;

    /**
     * Initialize servo hardware
     * @param hwMap Hardware map from OpMode
     */
    public void init(HardwareMap hwMap) {
        // Get servo from hardware map
        panServo = hwMap.get(Servo.class, "camera_pan");

        // Optional: If you have a tilt servo
        // tiltServo = hwMap.get(Servo.class, "camera_tilt");

        // Set to center position
        centerCamera();
    }

    /**
     * Initialize with both pan and tilt servos
     */
    public void init(HardwareMap hwMap, boolean useTilt) {
        panServo = hwMap.get(Servo.class, "camera_pan");

        if (useTilt) {
            tiltServo = hwMap.get(Servo.class, "camera_tilt");
        }

        centerCamera();
    }

    /**
     * Set pan servo position (left/right)
     * @param position 0.0 (left) to 1.0 (right)
     */
    public void setPan(double position) {
        // Clamp to valid range
        position = Math.max(PAN_MIN, Math.min(PAN_MAX, position));
        currentPan = position;
        panServo.setPosition(position);
    }

    /**
     * Set tilt servo position (up/down)
     * @param position 0.0 (down) to 1.0 (up)
     */
    public void setTilt(double position) {
        if (tiltServo != null) {
            position = Math.max(TILT_MIN, Math.min(TILT_MAX, position));
            currentTilt = position;
            tiltServo.setPosition(position);
        }
    }

    /**
     * Adjust pan by a small amount (for tracking)
     * @param delta Amount to change (-1.0 to 1.0)
     */
    public void adjustPan(double delta) {
        setPan(currentPan + delta);
    }

    /**
     * Adjust tilt by a small amount
     * @param delta Amount to change (-1.0 to 1.0)
     */
    public void adjustTilt(double delta) {
        setTilt(currentTilt + delta);
    }

    /**
     * Center the camera (point straight ahead)
     */
    public void centerCamera() {
        setPan(PAN_CENTER);
        setTilt(TILT_CENTER);
    }

    /**
     * Point camera at specific angle
     * @param bearing Horizontal angle in degrees (-90 to +90)
     */
    public void pointAtAngle(double bearing) {
        // Convert bearing angle to servo position
        // bearing = 0° → center (0.5)
        // bearing = -90° → full left (0.0)
        // bearing = +90° → full right (1.0)

        // Scale: -90° to +90° maps to 0.0 to 1.0
        double position = 0.5 + (bearing / 180.0);
        setPan(position);
    }

    /**
     * Track AprilTag with proportional control
     * @param bearing Horizontal angle to tag
     * @param range Distance to tag (for tilt calculation)
     * @param gain Control sensitivity (0.01 to 0.05 recommended)
     */
    public void trackTag(double bearing, double range, double gain) {
        // Calculate servo adjustment based on bearing error
        double panAdjustment = bearing * gain;

        // Smooth tracking - adjust current position slightly
        adjustPan(panAdjustment);

        // Optional: Calculate tilt based on distance
        if (tiltServo != null) {
            // Closer tag = look down, farther = look straight
            double tiltPosition = calculateTiltFromRange(range);
            setTilt(tiltPosition);
        }
    }

    /**
     * Calculate tilt angle based on distance to tag
     * @param range Distance in inches
     * @return Servo position (0.0 to 1.0)
     */
    private double calculateTiltFromRange(double range) {
        // Example: Close tags need camera tilted down
        // Adjust these values based on your camera mount height

        if (range < 12.0) {
            return 0.3;  // Look down
        } else if (range < 36.0) {
            return 0.5;  // Look straight
        } else {
            return 0.6;  // Look slightly up
        }
    }

    // Getters
    public double getCurrentPan() { return currentPan; }
    public double getCurrentTilt() { return currentTilt; }

    public boolean isAtLeftLimit() { return currentPan <= PAN_MIN + 0.01; }
    public boolean isAtRightLimit() { return currentPan >= PAN_MAX - 0.01; }
}