package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="Auto", group="Turtle Group")
public class AutoDistanceTest extends LinearOpMode {
    double tickToMMRatio = 0.561 / 1;
    double distance = 0;
    double desiredDistance = 1000;
    int startEncoderValue;
    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware_TT robotHardware = new RobotHardware_TT(this);
        robotHardware.init();
        List<Integer> encoderList = robotHardware.getEncoders();
        startEncoderValue = encoderList.get(0);
        while (opModeIsActive() && distance < desiredDistance) {
            robotHardware.mecanumDrive(0,0.25,0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue);
            telemetry.addData("Distance Driven", distance);
            telemetry.addData("Left Front Wheel", encoderList.get(0));
            telemetry.addData("Right Front Wheel", encoderList.get(1));
            telemetry.addData("Left Back Wheel", encoderList.get(2));
            telemetry.addData("Right Back Wheel", encoderList.get(3));
            telemetry.update();
        }

    }
}
