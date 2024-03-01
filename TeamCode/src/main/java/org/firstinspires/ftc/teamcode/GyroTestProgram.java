package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name="GyroTest", group="Turtle Group")
@Disabled

public class GyroTestProgram extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot.init();
        while (opModeIsActive()) {
            telemetry.addData("Angle:", robot.getYawAngles());
            telemetry.update();
        }
    }
}