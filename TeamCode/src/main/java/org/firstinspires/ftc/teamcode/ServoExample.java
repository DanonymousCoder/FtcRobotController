/*
* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestServo;

@TeleOp
public class ServoExample extends OpMode {
    TestServo bench = new TestServo();

    @Override
    public void init(){
        bench.init(hardwareMap);
    }
    @Override
    public void loop(){
        if (gamepad1.a){
            bench.setCamera(-1.0);
        }
        else {
            bench.setCamera(1.0);
        }

    }
}
*/