package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init();
        robot.resetImu();
        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();

            robot.resetEncoders();
            encoderList = robot.getEncoders();
            while (encoderList.get(0) > -100 && opModeIsActive()) {
                encoderList = robot.getEncoders();
                robot.mecanumDrive(0, 0.2, 0); //drive to the spike mark placing
                telemetry.addData("ticks", encoderList.get(0));
                telemetry.addData("Angle", robot.getYawAngles());
                telemetry.update();
            }
            robot.resetEncoders();
            encoderList = robot.getEncoders();
            while (encoderList.get(0) > -1330 && opModeIsActive()) {
                encoderList = robot.getEncoders();
                robot.mecanumDrive(-0.4, 0, 0); //drive to the spike mark placing
                telemetry.addData("ticks", encoderList.get(0));
                telemetry.addData("Angle", robot.getYawAngles());
                telemetry.update();
            }

            robot.stopAllMotors();
        }
    }
