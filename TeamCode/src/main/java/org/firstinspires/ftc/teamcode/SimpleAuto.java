package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name = "SimpleAuto", group = "Red Team")
@Disabled

public class SimpleAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    int startEncoderValue;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        robot.resetEncoders();
        waitForStart();
        robot.getEncoders();
        robot.autoStrafe(1300,-0.4);
    }
}
