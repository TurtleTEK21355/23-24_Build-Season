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
        boolean targetFound = false;

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        VisionPortal visionPortal;
        AprilTagDetection desiredTag = null;
        waitForStart();
        while (opModeIsActive()) {
            robot.autoDrive(100, 0.2);
            // Wait for the DS start button to be touched.
            waitForStart();
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    //HuskyLens stuff (Don't move before this block of code)
                    if (robot.blockLensX() >= 270 && robot.blockLensX() <= 280) {
                        if (robot.blockLensY() >= 60 && robot.blockLensY() <= 70) {
                            Tag = "Right";
                        }
                    } else if (robot.blockLensX() >= 170 && robot.blockLensX() <= 270) {
                        if (robot.blockLensY() >= 60 && robot.blockLensY() <= 70) {
                            Tag = "Center";
                        }
                    } else {
                        Tag = "Left";
                    }
                    telemetry.addData("\nTag: ", Tag);
                    telemetry.update();
                    //Desired Tag (Don't Move Before this code!)
                    if (Tag.equals("Left")) {
                        robot.DESIRED_TAG_ID = 4;
                    } else if (Tag.equals("Center")) {
                        robot.DESIRED_TAG_ID = 5;
                    } else if (Tag.equals("Right")) {
                        robot.DESIRED_TAG_ID = 6;
                    } else {
                        telemetry.addLine("Tag not found.");
                        telemetry.update();
                    }

                    // Here is where you would put code to place the pixel on the spike mark
                    robot.setIntake(-0.5);
                    sleep(10);
                    robot.setIntake(0);
                    robot.autoStrafe(100, -0.4); //don't know true numbers
                    robot.autoTurn(90, 0.2);
                    robot.autoDrive(100, 0.2); //don't know true numbers

                    targetFound = false;
                    desiredTag = null;
                    // Push telemetry to the Driver Station.
                    List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
                    //Makes list of what it currently sees
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            robot.mecanumDrive(0, 0, 0);
                            //  Check to see if we want to track towards this tag.
                            if ((robot.DESIRED_TAG_ID < 0) || (detection.id == robot.DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = detection;
                                break;
                            }
                            if ((robot.DESIRED_TAG_ID < 0 || detection.id == 4)) {
                                //if sees Red Left
                                if (robot.DESIRED_TAG_ID > 4) {
                                    robot.mecanumDrive(-0.5, 0, 0);
                                    //If the spike indicated Center or Right
                                }
                            } else if ((robot.DESIRED_TAG_ID < 0 || detection.id == 5)) {
                                //if sees Red Center
                                if (robot.DESIRED_TAG_ID > 5) {
                                    robot.mecanumDrive(-0.5, 0, 0);
                                    // if the spike indicated Right
                                } else if (robot.DESIRED_TAG_ID < 5) {
                                    robot.mecanumDrive(0.5, 0, 0);
                                    //if Spike indicated Left
                                }
                            } else if ((robot.DESIRED_TAG_ID < 0 || detection.id == 6)) {
                                //if sees Red Right
                                if (6 > robot.DESIRED_TAG_ID) {
                                    robot.mecanumDrive(0.5, 0, 0);
                                    //if Spike indicated Left or Center
                                }
                                // Here goes the scoring mechanism. It is hard to code, given that we don't know what mechanism we use.

                                else {
                                    // This tag is in the library, but we do not want to track it right now.
                                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);

                                }
                                if (targetFound == true) {
                                    while (robot.armMotorEncoders() < 100) {
                                        robot.setArm(0.2);
                                    }
                                    robot.setWrist(0.75); //also not true number
                                    robot.setClaw(0.44); //need true number
                                    while (robot.armMotorEncoders() > 0) {
                                        robot.setArm(-0.2);
                                    }
                                }

                            }
                        }
                    }
                    robot.autoStrafe(100,-0.4); //not true numbers
                    robot.autoDrive(100,0.2); //not true numbers
                }
            }
        }
    }
}
