package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name = "SimpleAuto", group = "Red Team")

public class SimpleAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;
    double tickToMMRatio = 0.561 / 1;
    double distance = 0;
    int startEncoderValue;
//19.2 * 28 = 96πmm <-- Replace these numbers.
//537.6 ticks = 301.6mm
// 1 tick = 0.561mm

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setPower(1);
        while (opModeIsActive()) {
            leftFrontDrive.setPower(0.1);
        }

    }
}