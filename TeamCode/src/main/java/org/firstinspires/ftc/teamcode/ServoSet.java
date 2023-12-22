package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Servos",group = "test")
public class ServoSet extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);
    @Override
    public void runOpMode(){
        robot.init();
        waitForStart();
        while (opModeIsActive()){
            robot.setClaw(0);
            robot.setWrist(0);
        }
    }
}
