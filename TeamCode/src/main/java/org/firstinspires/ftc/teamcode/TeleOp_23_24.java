package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="TeleOp")
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
                robot.launchServoGo(0.2);
                //need new numbers
            }


            robot.setWrist(Math.abs(gamepad2.left_stick_y));


            


//            if (gamepad2.right_bumper && gamepad2.left_bumper) {
//            } else if (gamepad2.left_bumper) {
//                robot.setWrist(0.25);
//            } else if (gamepad2.right_bumper) {
//                robot.setWrist(0);
//            }

            if (gamepad2.dpad_down && gamepad2.dpad_up) {
            } else if (gamepad2.dpad_up) {
                robot.setClaw(0.4);
            } else if (gamepad2.dpad_down) {
                robot.setClaw(0.68);
            }



            if ((gamepad2.y && gamepad2.a) || (gamepad2.y && gamepad2.b) || (gamepad2.a && gamepad2.b)) {
            } else if (gamepad2.a) {
                robot.setIntake(0.64);
            } else if (gamepad2.b) {
                robot.setIntake(-0.64);
            } else if (gamepad2.y) {
                robot.setIntake(0);
            }

            robot.setArm(-gamepad2.right_stick_y);



            if (-gamepad1.left_stick_x < 0) {
                Strafe = -Math.pow(-gamepad1.left_stick_x, 2);
            } else {
                Strafe = Math.pow(-gamepad1.left_stick_x, 2);
            }

//            if (-gamepad1.right_stick_x < 0) {
//                Turn = -Math.pow(-gamepad1.right_stick_x, 2);
//            } else {
//                Turn = Math.pow(-gamepad1.right_stick_x, 2);
//            }

            if(gamepad1.right_stick_x < 0){
                Turn = Turn + 1;
            }
            else if(gamepad1.right_stick_x > 0){
                Turn = Turn - 1;
            }
            if(Turn > 180){
                Turn = Turn - 360;
            }

            if(Turn < -180){
                Turn = Turn + 360;
            }

            telemetry.addData("Turn Value that is equal at the present moment to:", Turn);
            telemetry.addData("turning stick value", gamepad1.right_stick_x);
            telemetry.addData("Gyro", robot.getYawAngles());

            if (gamepad1.left_stick_y < 0) {
                Drive = -Math.pow(gamepad1.left_stick_y, 2);
            } else {
                Drive = Math.pow(gamepad1.left_stick_y, 2);
            }



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