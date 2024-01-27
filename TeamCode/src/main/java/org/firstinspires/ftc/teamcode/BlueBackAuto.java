package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="BlueBackAuto_Vision", group="Blue Vision Team")
public class BlueBackAuto extends LinearOpMode {
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
        waitForStart();

        long timer = 0;
        int y = robot.blockLensY();
        int x = robot.blockLensX();
        telemetry.addData("X: ", x);
        telemetry.addData("Y: ", y);
        telemetry.update();
        sleep(100);
        robot.autoDrive(100, 0.2);
        if (x >= 70 && x <= 120) {
            //Left
            // Here is where you would put code to place the pixel on the spike mark
            telemetry.addLine("Left");
            telemetry.update();
            robot.autoDrive(750, 0.2);
            robot.autoTurn(90,0.2);
            robot.autoDrive(150, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(-150,-0.2);
            robot.autoTurn(-90, 0.2);
            robot.autoDrive(-500, -0.2);
            robot.autoStrafe(-600, -0.2);
            robot.autoDrive(980,0.4);
            robot.autoStrafe(3500,0.7);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        } else if (x >= 180 && x <= 240) {
            //Center
            // Here is where code to place the pixel on the spike mark is.
            robot.autoDrive(750, 0.5);
            robot.autoStrafe(-50, -0.5);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(-150, -0.5);
            robot.autoStrafe(-450, -0.5);
            robot.autoDrive(980,0.4);
            robot.autoStrafe(3500,0.7);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}



        } else {
            //Right
            // Here is where you would put code to place the pixel on the spike mark
            robot.autoDrive(750, 0.2);
            robot.autoTurn(-90,0.2);
            robot.autoDrive(150, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(-50,-0.2);
            robot.autoTurn(90, 0.2);
            robot.autoDrive(-500,-0.2);
            robot.autoStrafe(-650, -0.2);
            robot.autoDrive(980,0.4);
            robot.autoStrafe(3500,0.7);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        }
    }
}