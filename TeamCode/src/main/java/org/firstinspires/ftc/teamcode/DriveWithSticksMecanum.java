package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MecanumTest", group="Turtle Group")
public class DriveWithSticksMecanum extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);
    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();
        boolean ToggleSpeed = false;
        boolean PreviousToggleReading = gamepad1.left_bumper;
        double Turn = 0;

        while (opModeIsActive()) {

            double Strafe = 0;
            double Drive = 0;


            

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
