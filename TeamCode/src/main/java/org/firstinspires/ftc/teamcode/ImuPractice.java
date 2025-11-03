package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchIMU;

@TeleOp
public class ImuPractice extends OpMode {

    TestBenchIMU bench = new TestBenchIMU();

    @Override
    public void init(){
      bench.init(hardwareMap);
    }

    @Override
    public void loop() {
      telemetry.addData("Heading" , bench.getHeading(AngleUnit.DEGREES));

    }
}
