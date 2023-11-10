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
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
            } else if (gamepad1.left_bumper) {
                robot.setWrist(0.25);
            } else if (gamepad1.right_bumper) {
                robot.setWrist(0);
            }
            if (gamepad1.dpad_down && gamepad1.dpad_up) {
            } else if (gamepad1.dpad_up) {
                robot.setClaw1(0.19);
            } else if (gamepad1.dpad_down) {
                robot.setClaw1(0.05);
            }
        }
    }
}