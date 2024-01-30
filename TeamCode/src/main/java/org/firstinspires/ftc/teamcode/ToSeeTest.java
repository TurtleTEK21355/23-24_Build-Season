package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.checkerframework.checker.units.qual.Angle;
import java.util.List;

@Autonomous(name="To See", group="Testing")
public class ToSeeTest extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm
    double tickToMMRatio = 0.561 / 1;
    int startEncoderValue;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.resetImu();
        robot.initLens();


        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();
        while (opModeIsActive()) {

            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    int y = robot.blockLensY();
                    int x = robot.blockLensX();
                    telemetry.addData("X: ", x);
                    telemetry.addData("Y: ", y);
                    telemetry.update();
                }

            }
        }
    }

}
