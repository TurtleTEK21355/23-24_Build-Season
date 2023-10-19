package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DroneLauncher", group = "Test")
public class DroneLaunchers extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    @Override
    public void runOpMode() {
        double speed = 0;
        robot.init();
        waitForStart();
        while (opModeIsActive()) {
            speed = -gamepad2.right_stick_y;
            robot.setArm(speed);

            if (gamepad2.left_bumper && gamepad2.right_bumper) {
            }
            else if (gamepad2.left_bumper) {
                robot.setWrist(0.5);
            }
            else if (gamepad2.right_bumper) {
                robot.setWrist(0);
            }
            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }
}
