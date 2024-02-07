package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Competition TeleOp", group="TeleOp")
public class TeleOp_23_24 extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);
    @Override
    public void runOpMode() {
        robot.init();
        robot.resetImu();

        waitForStart();
        boolean ToggleSpeed = false;
        boolean PreviousToggleReading = gamepad1.left_bumper;
        double Turn = 0;
        double Strafe = 0;
        double Drive = 0;
        double speed = 0.345;
        double wristPosition = 0.5;


        while (opModeIsActive()) {

            if (gamepad2.x && gamepad2.back) {
                robot.setLaunch();
                robot.checkVelocity();
                robot.launchServoGo(0.65);
                telemetry.addLine("LAUNCH THE NUKE!!!");
                sleep(1000);
                robot.setLaunchSpeed(0);
                robot.launchServoGo(0.53);
                //need new numbers
            }


//            if (gamepad2.right_trigger > 0.2 && wristPosition < 0.5) { // FILLER NUMBER STILL NEED THE ACTUAL NUMBERS)
//                wristPosition = wristPosition + 0.002;
//                telemetry.addLine("Move the wrist");
//            } else if (gamepad2.left_trigger > 0.2 && wristPosition > 0) { // FILLER NUMBER STILL NEED THE ACTUAL NUMBERS)
//                wristPosition = wristPosition - 0.002;
//            } else if (gamepad2.x) {
//                wristPosition = 0.5;
//            }
//            robot.setWrist(wristPosition);
//
//            //need new numbers!
//            if (gamepad2.left_bumper) {
//                // Closed
//                robot.setClaw(0.1); // FILLER NUMBER STILL NEED THE ACTUAL NUMBERS
//                telemetry.addLine("CLOSE CLAW");
//            }
//            else if (gamepad2.right_bumper || gamepad1.right_bumper) {
//                robot.setClaw(0.54); // FILLER NUMBER STILL NEED THE ACTUAL NUMBERS
//                telemetry.addLine("OPEN CLAW");
//            }


            if (gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2) {
            } else if (gamepad1.right_trigger > 0.2) {
                robot.setIntake(-0.4);
            } else if (gamepad1.left_trigger > 0.2) {
                robot.setIntake(0.4);

            } else {
                robot.setIntake(0);
            }

            robot.setArm(-gamepad2.right_stick_y);



            if (-gamepad1.left_stick_x < 0) {
                Strafe = -Math.pow(-gamepad1.left_stick_x, 2);
            } else {
                Strafe = Math.pow(-gamepad1.left_stick_x, 2);
            }


                        if (-gamepad1.right_stick_x < 0) {
                Turn = -Math.pow(-gamepad1.right_stick_x, 2);
            } else {
                Turn = Math.pow(-gamepad1.right_stick_x, 2);
            }

            if (-gamepad1.right_stick_x > 0) {
                Turn = Math.pow(-gamepad1.right_stick_x, 2);
            } else {
                Turn = -Math.pow(-gamepad1.right_stick_x, 2);
            }

            if(Turn > 180){
                Turn = Turn - 360;
                telemetry.addLine("Minus 360");
            }

            if(Turn < -180){
                Turn = Turn + 360;
                telemetry.addLine("Plus 360");
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
                robot.mecanumDrive(Strafe * 0.75,Drive * -0.75,Turn);
            }
            else {
                robot.mecanumDrive(Strafe * 0.5,Drive * -0.5, Turn);

            }


            //telemetry.addData("\nWrist: ", robot.wrist.getPosition());

            telemetry.update();



        }
    }
}