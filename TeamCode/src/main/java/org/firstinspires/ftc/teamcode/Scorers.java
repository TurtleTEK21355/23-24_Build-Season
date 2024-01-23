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
        double speed = -0.23;
        while (opModeIsActive()) {

            if (gamepad2.dpad_up) {
                speed = speed - 0.01;
                gamepad2.dpad_up = false;
            } else if (gamepad2.dpad_down) {
                speed = speed + 0.01;
                gamepad2.dpad_down = false;
            }

            robot.intake(speed);

            telemetry.addData("\nSpeed", speed);
            telemetry.update();
        }
    }
}