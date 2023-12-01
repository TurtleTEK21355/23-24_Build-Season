package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name="RedBackAuto", group="Turtle Group")
public class StrafeVsNormalDistanceTest extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

//To calibrate, drive the robot for 1000 ticks in the direction you want, and then divide that distance by 1000. This is how far you can go in one tick. Do the x and y axis seperately.
    //If I go for one tick, how far will I go on the y axis?
    double yAxisMovementToTicks;
    //If I go for one tick, how far will I go on the x axis?
    double xAxisMovementToTicks;
    double tickToMMRatio = 0.561 / 1;
    double distance = 0;
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

        while (opModeIsActive() && distance < 1000) {
            robotHardware.mecanumDrive(1, 0, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue);

        }

    }
}