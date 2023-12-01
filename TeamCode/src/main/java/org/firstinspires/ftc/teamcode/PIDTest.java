package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous()
public class PIDTest extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    public double error;
    public double previousError;
    public double proportional;
    public double integral;
    public double derivative;
    public double kP = 1;
    public double kI = 0;
    public double kD = 0;

    @Override
    public void runOpMode() throws InterruptedException {
    robot.init();

    waitForStart();

    while (opModeIsActive()) {
            double inputValue = robot.getYawAngles();
            double outputValue;
            error = inputValue;
            proportional = error;
            integral = integral + error;
            derivative = error - previousError;
            outputValue = kP * proportional + kI * integral + kD * derivative;
            robot.mecanumDrive(0, 1, outputValue);
            previousError = error;
        }
    }
}