package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="BlueFrontAuto_Vision", group="Blue Vision Team")
public class BlueFrontAuto extends LinearOpMode {
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
        robot.initLens();
        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        long timer = 0;
        waitForStart();

        int y = robot.blockLensY();
        int x = robot.blockLensX();
        telemetry.addData("X: ", x);
        telemetry.addData("Y: ", y);
        telemetry.update();
        sleep(100);
        robot.autoDrive(100, 0.2);
        if (x > 0 && x <= 120) {
            //Left
            // Here is where you would put code to place the pixel on the spike mark
            telemetry.addLine("Left");
            telemetry.update();
            robot.autoDrive(700, 0.2); //don't know true numbers
            robot.autoTurn(90,0.4);
            robot.autoDrive(100, 0.2);
            robot.setIntake(-0.22);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(-150,-0.2);
            robot.autoTurn(-90, 0.2);
            robot.autoDrive(-550,-0.2);
            robot.autoStrafe(1200, 0.4); //don't know true numbers
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        } else if (x >= 121 && x <= 280) {
            //Center
            // Here is where code to place the pixel on the spike mark is.
            telemetry.addLine("Center");
            telemetry.update();
            robot.autoDrive(725, 0.2);
            robot.autoStrafe(-50, -0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoStrafe(200, 0.4);
            robot.autoDrive(-600, -0.2);
            robot.autoStrafe(1200, 0.4);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        } else {
            //Right
            // Here is where you would put code to place the pixel on the spike mark
            //need numbers
            telemetry.addLine("Right");
            telemetry.update();
            robot.autoDrive(700, 0.2); //don't know true numbers
            robot.autoTurn(-90,0.4);
            robot.autoDrive(200, 0.2);
            robot.setIntake(-0.23);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +1000){}
            robot.setIntake(0);
            robot.autoDrive(-220,-0.2);
            robot.autoTurn(90, 0.2);
            robot.autoStrafe(100,0.2);
            robot.autoDrive(-550,-0.2);
            robot.autoStrafe(1200, 0.4); //don't know true numbers
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        }
    }
}
