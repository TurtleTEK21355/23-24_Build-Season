package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name="RedFrontAuto", group="Turtle Group")
public class RedFrontAuto extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);


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

        //if it reads Spike Mark in the left position
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive(0, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        robot.imuTurn(30);
        //place pixel
        robot.imuTurn(-120);
        while (opModeIsActive() && distance < 18) {
            robotHardware.mecanumDrive(0.25, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        //place pixel
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive( 1, 0, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }

        //if it reads Spike Mark in the center position
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive(0, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        //place pixel
        robot.imuTurn(-90);
        while (opModeIsActive() && distance < 18) {
            robotHardware.mecanumDrive(0, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        //place pixel
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive( 1, 0, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        //if it reads Spike Mark in the left position
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive(0, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        robot.imuTurn(30);
        //place pixel
        robot.imuTurn(-120);
        while (opModeIsActive() && distance < 18) {
            robotHardware.mecanumDrive(-0.25, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        //place pixel
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive( -1, 0, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }

        //if it reads Spike Mark in the left position
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive(0, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        robot.imuTurn(-30);
        //place pixel
        robot.imuTurn(-60);
        while (opModeIsActive() && distance < 18) {
            robotHardware.mecanumDrive(-0.25, 1, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }
        robot.resetEncoders();
        //place pixel
        while (opModeIsActive() && distance < 12) {
            robotHardware.mecanumDrive( 1, 0, 0);
            encoderList = robotHardware.getEncoders();
            distance = (encoderList.get(0) - startEncoderValue) * tickToMMRatio;
        }

    }



}