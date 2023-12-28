package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="RedFrontAuto", group="Red Team")
public class RedFrontAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);


    double tickToMMRatio = 0.561 / 1;
    double distance = 0;
    int startEncoderValue;
    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init();
        robot.getEncoders();
        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();

        while (opModeIsActive()) {
            robot.resetEncoders();
            while (encoderList.get(0) > -830 && opModeIsActive()) {
                encoderList = robot.getEncoders();
                robot.mecanumDrive(0, 0.5, 0); //drive to the spike mark placing
                telemetry.addData("ticks", encoderList.get(0));
                telemetry.update();
            }
            robot.mecanumDrive(0,0,0);
            robot.imuTurn(90);
            robot.resetEncoders();
            while (encoderList.get(0) > -1700 && opModeIsActive()) {
                encoderList = robot.getEncoders();
                robot.mecanumDrive(0, 0.5, 0); //drive to backdrop
                telemetry.addData("ticks", encoderList.get(0));
                telemetry.update();
            }
//                robotHardware.resetEncoders();
//                robotHardware.mecanumDrive(0,0,30);// filler value, need to have vision to determine how much to turn.
//                //places pixel
//                robotHardware.resetEncoders();
//                robotHardware.mecanumDrive(0,0,-30);// filler value, will be the opposite of the line above it,
//                robotHardware.resetEncoders();
//                while (encoderList.get(0) > 1000 && opModeIsActive()) {  //Parking
//                    robotHardware.mecanumDrive(0.5, -0.2, 0);
//                }
//                robotHardware.resetEncoders();
//                robotHardware.mecanumDrive(0,0,0);
//
        }
    }
}