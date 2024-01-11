package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.checkerframework.checker.units.qual.Angle;
import java.util.List;

@Autonomous(name="RedFrontAuto", group="Red Team")
public class RedFrontAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm
    double tickToMMRatio = 0.561 / 1;
    int startEncoderValue;

    String Tag = "Unseen";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initLens();
        robot.init();
        robot.initAprilTag();
        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);

        waitForStart();
        while (opModeIsActive()) {
            //HuskyLens stuff (Don't move before this block of code)

            robot.autoDrive(100, 0.2);
            robot.autoStrafe(1300, -0.4);
        }
    }
}
