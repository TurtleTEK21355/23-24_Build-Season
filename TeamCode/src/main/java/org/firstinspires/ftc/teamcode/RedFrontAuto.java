package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.checkerframework.checker.units.qual.Angle;
import java.util.List;

@Autonomous(name="RedFrontAuto", group="Red Team")
public class RedFrontAuto extends LinearOpMode {
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
        while (opModeIsActive()) {
            robot.autoDrive(100, 0.2);
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    robot.blockLensY();
                    robot.blockLensX();
                    //HuskyLens stuff (Don't move before this block of code)
                    if (robot.blockLensX() >= 270 && robot.blockLensX() <= 280) {
                        if (robot.blockLensY() >= 60 && robot.blockLensY() <= 70) {
                            //Left
                            // Here is where you would put code to place the pixel on the spike mark
                            //need numbers
                            robot.autoStrafe(100, -0.2); //don't know true numbers
                            robot.autoDrive(100, 0.2); //don't know true numbers
                            robot.setIntake(-0.5);
                            sleep(10);
                            robot.setIntake(0);
                            robot.autoStrafe(100, -0.4); //don't know true numbers
                            robot.autoTurn(90, 0.2);
                            robot.autoDrive(100, 0.2); //don't know true numbers
                        } else {
                            //Right
                            // Here is where you would put code to place the pixel on the spike mark
                            //need numbers
                            robot.autoStrafe(100, 0.2); //don't know true numbers
                            robot.autoDrive(100, 0.2); //don't know true numbers
                            robot.setIntake(-0.5);
                            sleep(10);
                            robot.setIntake(0);
                            robot.autoStrafe(100, -0.4); //don't know true numbers
                            robot.autoTurn(90, 0.2);
                            robot.autoDrive(100, 0.2); //don't know true numbers
                        }
                    } else if (robot.blockLensX() >= 170 && robot.blockLensX() <= 270) {
                        if (robot.blockLensY() >= 60 && robot.blockLensY() <= 70) {
                            //Center
                            // Here is where you would put code to place the pixel on the spike mark
                            //need numbers
                            robot.autoDrive(200, 0.2);
                            robot.setIntake(-0.5);
                            sleep(10);
                            robot.setIntake(0);
                            robot.autoStrafe(100, -0.4); //don't know true numbers
                            robot.autoTurn(90, 0.2);
                            robot.autoDrive(100, 0.2); //don't know true numbers
                        } else {
                            //Right
                            // Here is where you would put code to place the pixel on the spike mark
                            //need numbers
                            robot.autoStrafe(100, 0.2); //don't know true numbers
                            robot.autoDrive(100, 0.2); //don't know true numbers
                            robot.setIntake(-0.5);
                            sleep(10);
                            robot.setIntake(0);
                            robot.autoStrafe(100, -0.4); //don't know true numbers
                            robot.autoTurn(90, 0.2);
                            robot.autoDrive(100, 0.2); //don't know true numbers
                        }
                    } else {
                        //Right
                        // Here is where you would put code to place the pixel on the spike mark
                        //need numbers
                        robot.autoStrafe(100, 0.2); //don't know true numbers
                        robot.autoDrive(100, 0.2); //don't know true numbers
                        robot.setIntake(-0.5);
                        sleep(10);
                        robot.setIntake(0);
                        robot.autoStrafe(100, -0.4); //don't know true numbers
                        robot.autoTurn(90, 0.2);
                        robot.autoDrive(100, 0.2); //don't know true numbers
                    }


                    robot.autoStrafe(100,-0.4); //not true numbers
                    robot.autoDrive(100,0.2); //not true numbers

                }
            }
        }
    }
}