package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="HuskyTest", group="Turtle Group")
public class HuskeyLensTest extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);
    double y;
    double x;
    String Tag;
    @Override
    public void runOpMode() {
        robot.init();

        Deadline rateLimit = new Deadline(robot.READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!robot.huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + robot.huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = robot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                y = blocks[i].y;
                x = blocks[i].x;
                telemetry.addData("\nX:", blocks[i].x);
                telemetry.addData("\nY:", blocks[i].y);
                telemetry.addData("Blocks: ", blocks[i].toString());
            }

            if (x != 0 && y != 0) {
                robot.mecanumDrive(0,-1,0);
                sleep(50);
                robot.mecanumDrive(0,0,0);
            }

           /* if (x >= 270 && x <= 280) {
                if (y >= 60 && y <= 70) {
                    Tag = "Right";
                }
            } else if (x >= 170 && x <= 270) {
                if (y >= 60 && y <= 70) {
                    Tag = "Center";
                }
            } else {
                Tag = "Left";
            }
            telemetry.addData("\nTag: ", Tag);*/


            telemetry.update();
        }
    }

}
