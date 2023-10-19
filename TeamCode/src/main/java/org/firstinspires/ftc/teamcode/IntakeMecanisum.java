package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name = "Intakes", group = "Test")
public class IntakeMecanisum extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    @Override
    public void runOpMode(){
        double claw1Position = 0;
        robot.init();
        waitForStart();
        while (opModeIsActive()) {
           /* if (gamepad2.left_bumper && gamepad2.right_bumper) {
            }
            else if (gamepad2.right_bumper) {
                claw1Position = claw1Position + 0.01;
                robot.setClaw1(claw1Position);
                gamepad2.right_bumper = false;
            }
            else if (gamepad2.left_bumper) {
                claw1Position = 0;
                robot.setClaw1(claw1Position);
                gamepad2.left_bumper = false;
            }

            if (gamepad2.a && gamepad2.b){
            }
            else if (gamepad2.a) {
                robot.setWrist(0);
            }
            else if (gamepad2.b) {
                robot.setWrist(.5);
            }*/

            /*robot.setArm(gamepad2.left_stick_y);


            telemetry.addData("Claw1", claw1Position);
            telemetry.update();*/

        }
    }
}
