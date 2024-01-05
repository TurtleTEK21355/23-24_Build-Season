package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test1", group = "Test")
public class MotorTestSpace extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    @Override
    public void runOpMode(){
        //double claw1Position = 0;
        robot.init();
        waitForStart();
        double speed = 0.68;
        while (opModeIsActive()) {
            if (gamepad2.left_bumper && gamepad2.right_bumper) {
            }
            else if (gamepad2.right_bumper) {
                robot.setClaw(0.4);
                gamepad2.right_bumper = false;
            }
            else if (gamepad2.left_bumper) {
                robot.setClaw(0.68);
                gamepad2.right_bumper = false;
            }


//            if (gamepad2.a && gamepad2.b){
//            }
//            else if (gamepad2.a) {
//                robot.setWrist(0);
//            }
//            else if (gamepad2.b) {
//                robot.setWrist(.5);
//            }

            robot.setWrist(gamepad1.right_stick_y);

            telemetry.addData("Left Claw Position: ",robot.clawLeft.getPosition());
            telemetry.addData("Right Claw Position: ",robot.clawRight.getPosition());


            robot.intake(speed);

            robot.setArm(gamepad2.right_stick_y * 0.5);
            telemetry.addData("Speed of Arm: ", gamepad2.right_stick_y * 0.5);
            telemetry.addData("Speed of Intake: ", speed);
            telemetry.update();

        }
    }
}
