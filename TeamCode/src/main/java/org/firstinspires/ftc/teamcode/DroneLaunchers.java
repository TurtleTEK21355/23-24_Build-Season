package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DroneLauncher", group = "Test")
public class DroneLaunchers extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    @Override
    public void runOpMode() {
        double speed = -0.32;
        robot.init();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_down && gamepad2.dpad_up) {
            } else if (gamepad2.dpad_up) {
                speed = speed + 0.01;
            } else if (gamepad2.dpad_down) {
                speed = speed - 0.01;
            }
            robot.goLaunch(gamepad2.right_stick_y);
            telemetry.addData("Launch Servo Position: ", robot.flick.getPosition());
           // speed = -gamepad2.right_stick_y;
            robot.setArm(speed);

           /* if (gamepad2.left_bumper && gamepad2.right_bumper) {
            }
            else if (gamepad2.left_bumper) {
                robot.setWrist(0.5);
            }
            else if (gamepad2.right_bumper) {
                robot.setWrist(0);
            }*/
            telemetry.addData("\nSpeed", speed);
            telemetry.update();
        }
    }
}
