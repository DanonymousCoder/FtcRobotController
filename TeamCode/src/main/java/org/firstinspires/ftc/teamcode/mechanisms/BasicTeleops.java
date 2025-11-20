package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench;

@TeleOp(name="Basic Drive", group="TeleOp")
public class BasicTeleops  extends OpMode {

    TestBench bench = new TestBench();

    @Override
    public void init() {
        // Initialize hardware
        bench.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Left Stick = Drive");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get gamepad input
        double drive = -gamepad1.left_stick_y;   // Forward/backward (inverted)
        double turn = gamepad1.left_stick_x;     // Left/right turning

        // Calculate motor powers
        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Set motor powers
        bench.setLeft_driveSpeed(leftPower);
        bench.setRight_driveSpeed(rightPower);

        // Display telemetry
        telemetry.addData("Left Stick Y", "%.2f", drive);
        telemetry.addData("Left Stick X", "%.2f", turn);
        telemetry.addData("Left Power", "%.2f", leftPower);
        telemetry.addData("Right Power", "%.2f", rightPower);
        telemetry.update();
    }
}