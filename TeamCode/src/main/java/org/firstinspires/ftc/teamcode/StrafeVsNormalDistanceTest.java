package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name = "StrafeVsNormalDistanceTest", group = "Turtle Group")
@Disabled
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

        robot.init();
        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);

        waitForStart();

        robot.autoDrive(1000,0.2);
        robot.stopAllMotors();

    }
}