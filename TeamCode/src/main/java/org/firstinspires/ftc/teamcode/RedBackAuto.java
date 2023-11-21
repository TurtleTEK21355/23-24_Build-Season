package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;

@Autonomous(name="RedBackAuto", group="Turtle Group")
public class RedBackAuto extends LinearOpMode {
    RobotHardware_TT   robot       = new RobotHardware_TT(this);


    @Override
    public void runOpMode() {
        robot.initAuto();
    }


}