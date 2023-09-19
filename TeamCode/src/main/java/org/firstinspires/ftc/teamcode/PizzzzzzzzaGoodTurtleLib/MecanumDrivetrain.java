package org.firstinspires.ftc.teamcode.PizzzzzzzzaGoodTurtleLib;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrivetrain {
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;


    public MecanumDrivetrain(DcMotor rightFrontDrive, DcMotor leftFrontDrive, DcMotor rightBackDrive, DcMotor leftBackDrive) {
        this.leftFrontDrive = leftFrontDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.rightBackDrive = rightBackDrive;
        this.leftBackDrive = leftBackDrive;
    }

    public void drive(double forwardPower, double strafePower, double turnPower) {
        leftFrontDrive.setPower(forwardPower + strafePower + turnPower);
        rightFrontDrive.setPower(forwardPower - strafePower - turnPower);
        leftBackDrive.setPower(forwardPower - strafePower + turnPower);
        rightBackDrive.setPower(forwardPower + strafePower - turnPower);
    }
}