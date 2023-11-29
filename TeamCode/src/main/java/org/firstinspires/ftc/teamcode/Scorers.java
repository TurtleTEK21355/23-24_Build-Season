package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Scorers", group = "Test")
public class Scorers extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        while (opModeIsActive()) {

            robot.setWrist(gamepad1.left_stick_y);
        }
    }
}