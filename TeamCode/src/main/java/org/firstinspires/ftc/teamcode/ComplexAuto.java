//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//import java.util.List;
//
//@Autonomous(name="ComplexAuto", group="Turtle Group")
//    public class ComplexAuto extends LinearOpMode {
//    RobotHardware_TT robot = new RobotHardware_TT(this);
//
//
//    double tickToMMRatio = 0.561 / 1;
//    double distance = 0;
//    int startEncoderValue;
//    String Tag = "Unseen";
//    //19.2 * 28 = 96πmm <-- Replace these numbers.
//    //537.6 ticks = 301.6mm
//    // 1 tick = 0.561mm
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        RobotHardware_TT robotHardware = new RobotHardware_TT(this);
//        robotHardware.init();
//        robotHardware.getEncoders();
//        List<Integer> encoderList = robotHardware.getEncoders();
//        startEncoderValue = encoderList.get(0);
//        robot.initLens();
//        VisionPortal visionPortal;
//        AprilTagDetection desiredTag = null;
//        runOpMode(); {
//            robot.init();
//            boolean targetFound = false;
//            robot.initAprilTag();
//            // Wait for the DS start button to be touched.
//            waitForStart();
//            if (opModeIsActive()) {
//                while (opModeIsActive()) {
//                    //HuskyLens stuff (Don't move before this block of code)
//                    if (robot.blockLensX() >= 270 && robot.blockLensX() <= 280) {
//                        if (robot.blockLensY() >= 60 && robot.blockLensY() <= 70) {
//                            Tag = "Right";
//                        }
//                    } else if (robot.blockLensX() >= 170 && robot.blockLensX() <= 270) {
//                        if (robot.blockLensY() >= 60 && robot.blockLensY() <= 70) {
//                            Tag = "Center";
//                        }
//                    } else {
//                        Tag = "Left";
//                    }
//                    telemetry.addData("\nTag: ", Tag);
//                    telemetry.update();
//                    //Desired Tag (Don't Move Before this code!)
//                    if (Tag.equals("Left")) {
//                        robot.DESIRED_TAG_ID = 4;
//                    } else if (Tag.equals("Center")) {
//                        robot.DESIRED_TAG_ID = 5;
//                    } else if (Tag.equals("Right")) {
//                        robot.DESIRED_TAG_ID = 6;
//                    } else {
//                        telemetry.addLine("Tag not found.");
//                        telemetry.update();
//                    }
//
//                    // Here is where you would put code to place the pixel on the spike mark
//
//                    targetFound = false;
//                    desiredTag = null;
//                    // Push telemetry to the Driver Station.
//                    List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();
//                    //Makes list of what it currently sees
//                    for (AprilTagDetection detection : currentDetections) {
//                        // Look to see if we have size info on this tag.
//                        if (detection.metadata != null) {
//                            robot.mecanumDrive(0, 0, 0);
//                            //  Check to see if we want to track towards this tag.
//                            if ((robot.DESIRED_TAG_ID < 0) || (detection.id == robot.DESIRED_TAG_ID)) {
//                                // Yes, we want to use this tag.
//                                targetFound = true;
//                                desiredTag = detection;
//                                break;
//                            }
//                            if ((robot.DESIRED_TAG_ID < 0 || detection.id == 4)) {
//                                //if sees Red Left
//                                if (robot.DESIRED_TAG_ID > 4) {
//                                    robot.mecanumDrive(-0.5, 0, 0);
//                                    //If the spike indicated Center or Right
//                                }
//                            } else if ((robot.DESIRED_TAG_ID < 0 || detection.id == 5)) {
//                                //if sees Red Center
//                                if (robot.DESIRED_TAG_ID > 5) {
//                                    robot.mecanumDrive(-0.5, 0, 0);
//                                    // if the spike indicated Right
//                                } else if (robot.DESIRED_TAG_ID < 5) {
//                                    robot.mecanumDrive(0.5, 0, 0);
//                                    //if Spike indicated Left
//                                }
//                            } else if ((robot.DESIRED_TAG_ID < 0 || detection.id == 6)) {
//                                //if sees Red Right
//                                if (6 > robot.DESIRED_TAG_ID) {
//                                    robot.mecanumDrive(0.5, 0, 0);
//                                    //if Spike indicated Left or Center
//                                }
//                            while (robot.armMotorEncoders() > 100) {
//                                //^^^Test Value for Encoders^^^
//                                    robotHardware.setArm(0.3333);
//                            }
//                            // Here goes the scoring mechanism. It is hard to code, given that we don't know what mechanism we use.
//
//                            //else if () {
//                                // This tag is in the library, but we do not want to track it right now.
//                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                            }
//                        }
//                    }
//                    if (targetFound) {
//                        telemetry.addLine("\nFound!");
//                        telemetry.addData("\n", robot.DESIRED_TAG_ID);
//                        robot.mecanumDrive(0, 0, 0);
//                        //sends a the robot sees the correct tag
//                        //Put code here to place pixel on backdrop
//                    } else {
//                        telemetry.addLine("\nNot found.");
//                    }
//                    robot.telemetryAprilTag();
//                    // Save CPU resources; can resume streaming when needed.
//                    // if (gamepad1.dpad_down) {
//                    //    visionPortal.stopStreaming();
//                    ///  } else if (gamepad1.dpad_up) {
//                    //   visionPortal.resumeStreaming();
//                    //      }
//                    telemetry.update();
//                    // Share the CPU.
//                    sleep(20);
//                }
//            }
//            // Save more CPU resources when camera is no longer needed.
//            robot.visionPortal.close();
//        }
//    }
//}