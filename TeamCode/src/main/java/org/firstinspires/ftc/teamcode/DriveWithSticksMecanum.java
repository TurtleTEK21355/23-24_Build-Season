package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Wheel testing TeleOp", group="Turtle Group")
public class DriveWithSticksMecanum extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);
   // PorportionalController    p    = new PorportionalController();
   // PorportionalController   elevatorP = new PorportionalController();
    @Override
    public void runOpMode() {
        robot.init();
        robot.resetImu();
        waitForStart();
        boolean ToggleSpeed = false;
        boolean PreviousToggleReading = gamepad1.left_bumper;
        double Turn = 0;

        while (opModeIsActive()) {

            double Strafe = 0;
            double Drive = 0;

            if (gamepad1.y) {

            }

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
                Turn = Turn - Math.abs(gamepad1.right_stick_x * 2);
                telemetry.addData("Current turn value, should be subtracting", Turn);
            }
            else if(gamepad1.right_stick_x > 0){
                Turn = Turn + Math.abs(gamepad1.right_stick_x * 2);
                telemetry.addData("Current turn value, should be adding", Turn);
            }

            if(Turn > 180){
                Turn = Turn - 360;
                telemetry.addLine("Minus 360");
            }

            if(Turn < -180){
                Turn = Turn + 360;
                telemetry.addLine("Plus 360");
            }

            telemetry.addData("Turn Value", Turn);
            telemetry.addData("turning stick value", gamepad1.right_stick_x);
            telemetry.addData("Gyro", robot.getYawAngles());

            if (gamepad1.left_stick_y < 0) {
                Drive = Math.pow(gamepad1.left_stick_y, 2);
            } else {
                Drive = -Math.pow(gamepad1.left_stick_y, 2);
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
