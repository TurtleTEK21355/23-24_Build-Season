package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="RedBackAuto_Vision", group="Red Vision Team")
public class RedBackAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm
    double tickToMMRatio = 0.561 / 1;
    int startEncoderValue;



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();

        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();

        long timer = 0;

        //Seeing stuff here

        sleep(100);
        robot.autoDrive(100, 0.2);
        if () {
            //Left
            // Here is where you would put code to place the pixel on the spike mark
            telemetry.addLine("Left");
            telemetry.update();
            robot.autoDrive(725, 0.2);
            robot.autoTurn(90,0.2);
            robot.autoDrive(150, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(-150,-0.2);
            robot.autoTurn(-90, 0.2);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(650, 0.3);
            robot.autoDrive(950,0.3);
            robot.autoStrafe(-3650,-0.5);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        } else if () {
            //Center
            // Here is where code to place the pixel on the spike mark is.

            robot.autoDrive(725, 0.2);
            robot.autoStrafe(50, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(450, 0.2);
            robot.autoDrive(950,0.3);
            robot.autoStrafe(-3650,-0.5);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}

        } else {
            //Right
            // Here is where you would put code to place the pixel on the spike mark
            //need numbers
            robot.autoDrive(725, 0.2);
            robot.autoTurn(-90,0.2);
            robot.autoDrive(100, 0.2);
            robot.setIntake(-0.25);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 1000){}
            robot.setIntake(0);
            robot.autoDrive(-200,-0.2);
            robot.autoTurn(90, 0.2);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(450, 0.2);
            robot.autoDrive(950,0.3);
            robot.autoStrafe(-3650,-0.5);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        }
    }
}