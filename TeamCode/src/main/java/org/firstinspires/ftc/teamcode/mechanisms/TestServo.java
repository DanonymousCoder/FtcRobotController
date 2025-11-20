/*
* package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo {
    private Servo camera;

    public void init(HardwareMap hw+Map) {
        camera = hwMap.get(Servo.class, "camera");
    }

    public void setCamera(double angle){
        camera.setPosition(angle);
    }

}

* */