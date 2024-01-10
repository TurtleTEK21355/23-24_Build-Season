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

@Autonomous(name = "RedBackAuto", group = "Red Team")
public class RedBackAuto extends LinearOpMode {
RobotHardware_TT robot = new RobotHardware_TT(this);

double tickToMMRatio = 0.561 / 1;
int startEncoderValue;

@Override
public void runOpMode() throws InterruptedException {

    robot.init();
    robot.resetImu();

    List<Integer> encoderList = robot.getEncoders();
    startEncoderValue = encoderList.get(0);

    waitForStart();

    robot.autoDrive(1650, 0.2);
    robot.autoStrafe(3350, -0.2);
    }
}