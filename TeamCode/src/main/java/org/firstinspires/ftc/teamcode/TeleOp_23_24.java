package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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



        while (opModeIsActive()) {

            if (gamepad2.x && gamepad2.back) {
                robot.setLaunch();
                robot.checkVelocity();
                robot.launchServoGo(0.65);
                robot.checkVelocity();
                telemetry.addLine("LAUNCH THE NUKE!!!");
                sleep(1000);
                robot.setLaunchSpeed(0);
                robot.launchServoGo(0.53);
            }
            robot.launchNumbers();

            


            if (gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2) {
            } else if (gamepad1.right_trigger > 0.2) {
                robot.setIntake(0.45);
            } else if (gamepad1.left_trigger > 0.2) {
                robot.setIntake(-0.25);
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




            telemetry.update();



        }
    }
}