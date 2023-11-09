package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="Auto", group="Turtle Group")
public class AutoDistanceTest extends LinearOpMode {
    double tickToMMRatio = 0.561 / 1;
    double distance = 0;
    double desiredDistance = 25000;
    int startEncoderValue;
    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware_TT robotHardware = new RobotHardware_TT(this);
        robotHardware.init();
        robotHardware.getEncoders();
        List<Integer> encoderList = robotHardware.getEncoders();
        startEncoderValue = encoderList.get(0);
        while (opModeIsActive() && distance < desiredDistance) {
            robotHardware.mecanumDrive(0,1,0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }

    }
}
