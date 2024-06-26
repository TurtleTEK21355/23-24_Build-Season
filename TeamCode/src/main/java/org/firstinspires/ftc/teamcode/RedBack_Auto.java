package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="RedBackAuto", group="Red Team")
@Disabled

public class RedBack_Auto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm
    double tickToMMRatio = 0.561 / 1;
    int startEncoderValue;

    String Tag = "Unseen";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();

        robot.autoDrive(1650, 0.2);
        robot.autoStrafe(-3250, -0.4);
    }
}

