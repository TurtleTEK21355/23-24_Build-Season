package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.Angle;

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
        robot.resetImu();
        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();

        while (opModeIsActive()) {
            double startingTicks = encoderList.get(0);
            telemetry.addData("starting ticks", startingTicks);
            telemetry.update();
            sleep(3000);
            robot.resetEncoders();
            double afterResettingTicks = encoderList.get(0);
            telemetry.addData("post reset ticks", afterResettingTicks);
            telemetry.update();
            sleep(3000);
            while (encoderList.get(0) > -830 && opModeIsActive()) {
                encoderList = robot.getEncoders();
                robot.mecanumDrive(0, -0.2, 0); //drive to the spike mark placing
                telemetry.addData("ticks", encoderList.get(0));
                telemetry.addData("Angle", robot.getYawAngles());
                telemetry.update();
            }

            robot.stopAllMotors();
//            double tix = encoderList.get(0);
//            while (opModeIsActive()) {
//                encoderList = robot.getEncoders();
//                telemetry.addData("pre-tix", tix);
//                telemetry.addData("tix", encoderList.get(0));
//                telemetry.addData("Angele", robot.getYawAngles());
//                telemetry.update();
//            }

           // scan
           // turn to right place
           // place pixel
            robot.stopAllMotors();
            while (robot.getYawAngles() > -92 && opModeIsActive()){
                robot.mecanumTurn(-90,0.5);
                telemetry.addLine("NOT GOING STRATE REELY BAD CRY AND KIKK AND SCREEM");
                telemetry.addData("ticks", encoderList.get(0));
                telemetry.addData("startingTicks", startingTicks);
                telemetry.addData("Angle", robot.getYawAngles());
                telemetry.update();
            }
            robot.stopAllMotors();
//            robot.resetEncoders();
//            while (encoderList.get(0) > -1600 && opModeIsActive()) {
//                encoderList = robot.getEncoders();
//                robot.mecanumDrive(0, 0.5, 0); //drive to backdrop

//                telemetry.addData("ticks", encoderList.get(0));
//                telemetry.update();
//            }
//            robot.mecanumDrive(0,0,0);
//            robot.resetEncoders();
//            robot.mecanumDrive(0,0,30);// filler value, need to have vision to determine how much to turn.
//            //places pixel
//            robot.resetEncoders();
//            robot.mecanumDrive(0,0,-30);// filler value, will be the opposite of the line above it,
//            robot.resetEncoders();
//            while (encoderList.get(0) > 1000 && opModeIsActive()) {  //Parking
//                robot.mecanumDrive(0.5, -0.2, 0);
//            }
//            robot.resetEncoders();
//            robot.mecanumDrive(0,0,0);
//            return;
        }
    }
}