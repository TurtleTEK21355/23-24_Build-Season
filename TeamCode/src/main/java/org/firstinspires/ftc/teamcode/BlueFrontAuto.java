package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.util.List;

@Autonomous(name="BlueFrontAuto", group="Blue Team")
public class BlueFrontAuto extends LinearOpMode {


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
            while (encoderList.get(0) < 1330 && opModeIsActive()) {
                encoderList = robot.getEncoders();
                robot.mecanumDrive(0.4, 0, 0); //drive to the spike mark placing
                telemetry.addData("ticks", encoderList.get(0));
                telemetry.addData("Angle", robot.getYawAngles());
                telemetry.update();
            }

            robot.stopAllMotors();
        }
    }