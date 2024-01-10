package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name = "SimpleAuto", group = "Red Team")

public class SimpleAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    int startEncoderValue;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.resetImu();
        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();
        robot.autoDrive(300,0.2);
        robot.stopAllMotors();
        robot.autoStrafe(250,-0.5);
        robot.autoTurn(90,0.2);
    }
}
