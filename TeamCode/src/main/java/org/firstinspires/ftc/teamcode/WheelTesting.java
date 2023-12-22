package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="WheelTester", group="Turtle Group")
public class WheelTesting extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    @Override
    public void runOpMode() {
        robot.init();


        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_up) {
                robot.runRightFrontMotor();
            }
            else if (gamepad1.dpad_right) {
                robot.runRightBackMotor();
            }
            else if (gamepad1.dpad_down) {
                robot.runLeftBackMotor();
            }
            else if (gamepad1.dpad_left) {
                robot.runLeftFrontMotor();
            }
            else if (gamepad1.b) {
                robot.mecanumDrive(0,0,0);
            }
        }
    }
}
