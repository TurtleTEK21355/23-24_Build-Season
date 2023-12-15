package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.checkerframework.checker.initialization.qual.Initialized;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.File;
import java.util.List;
@Autonomous(name="GyroTestingOpmode", group="Turtle Group")
public class GyroTestingOpmode extends LinearOpMode {
    private IMU scootImu;

    @Override
    public void runOpMode(){
        scootImu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        scootImu.initialize(new IMU.Parameters(orientationOnRobot));
        scootImu.resetYaw();
        waitForStart();
        while (opModeIsActive());{
            telemetry.addData("Angle:", scootImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addLine("65436787grdetsrtfyguyrtestfygutdyrstdtfuytiu65432");
            telemetry.update();
        }
    }


}