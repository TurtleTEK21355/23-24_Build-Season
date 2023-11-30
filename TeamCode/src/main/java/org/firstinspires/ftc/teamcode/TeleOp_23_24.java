package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MecanumTest", group="Turtle Group")
public class TeleOp_23_24 extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);
    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();
        boolean ToggleSpeed = false;
        boolean PreviousToggleReading = gamepad1.left_bumper;
        double Turn = 0;
        double Strafe = 0;
        double Drive = 0;
        double speed = 0.32;

        while (opModeIsActive()) {
            if (gamepad2.x && gamepad2.back) {
                robot.setLaunch(speed);
                sleep(30);
                robot.beginFlick(0.5);
            }


            robot.setWrist(Math.abs(gamepad2.right_stick_y));

            //for finger-claw
            if (gamepad2.dpad_left && gamepad2.dpad_right) {
            } else if (gamepad2.dpad_right) {
                robot.setClaw1(0.19);
            } else if (gamepad2.dpad_left) {
                robot.setClaw1(0.05);
            }

            if ((gamepad2.y && gamepad2.a) || (gamepad2.y && gamepad2.b) || (gamepad2.a && gamepad2.b)) {
            } else if (gamepad2.a) {
                robot.setIntake(0.5);
            } else if (gamepad2.b) {
                robot.setIntake(-0.5);
            } else if (gamepad2.y) {
                robot.setIntake(0);
            }

            robot.setArm(-gamepad2.right_stick_y);



            if (-gamepad1.left_stick_x < 0) {
                Strafe = -Math.pow(-gamepad1.left_stick_x, 2);
            } else {
                Strafe = Math.pow(-gamepad1.left_stick_x, 2);
            }

            if(gamepad1.right_stick_x < 0){
                Turn = Turn + 1;
            }
            else if(gamepad1.right_stick_x > 0){
                Turn = Turn - 1;
            }

            telemetry.addData("\nTurn Value that is equal at the present moment:", Turn);
            telemetry.addData("\nturning stick value", gamepad1.right_stick_x);

            if (gamepad1.left_stick_y < 0) {
                Drive = -Math.pow(gamepad1.left_stick_y, 2);
            } else {
                Drive = Math.pow(gamepad1.left_stick_y, 2);
            }

            /* Jordan Mode:
            if (gamepad1.left_trigger) {
                Drive = -Math.pow(gamepad1.left_trigger, 2);
            } else {
                Drive = Math.pow(gamepad1.right_trigger, 2);
            }*/

            if (!PreviousToggleReading && gamepad1.left_bumper) {
                ToggleSpeed = !ToggleSpeed;
            }
            PreviousToggleReading = gamepad1.left_bumper;


            if (ToggleSpeed == true) {
                robot.mecanumDrive(Strafe * 0.6, Drive * 0.6, Turn);
            }
            else {
                robot.mecanumDrive(Strafe * 0.3, Drive * 0.3, Turn);
            }


            telemetry.update();


        }
    }
}