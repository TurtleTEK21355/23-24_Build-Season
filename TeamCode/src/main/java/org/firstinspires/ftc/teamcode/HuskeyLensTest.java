package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="MecanumTest", group="Turtle Group")
public class HuskeyLensTest extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);
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
                telemetry.addData("Block", blocks[i].toString());
            }

            telemetry.update();
        }
    }

}
