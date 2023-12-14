package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.util.List;

@Autonomous(name="BlueBackAuto", group="Turtle Group")
public class BlueBackAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);


    double tickToMMRatio = 0.561 / 1;
    double distance = 0;
    int startEncoderValue;
    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware_TT robotHardware = new RobotHardware_TT(this);
        robotHardware.init();
        robotHardware.getEncoders();
        List<Integer> encoderList = robotHardware.getEncoders();
        startEncoderValue = encoderList.get(0);


/**
 * The variable to store our instance of the AprilTag processor.
 *//**
 * The variable to store our instance of the vision portal.
 */
        VisionPortal visionPortal;

        AprilTagDetection desiredTag = null;
            runOpMode (); {
            robot.init();
            boolean targetFound = false;
            robot.initAprilTag();
            // Wait for the DS start button to be touched.
            waitForStart();
            if (opModeIsActive()) {
                while (opModeIsActive()) {
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
                                break;  // don't look any further.
                            }
                            if ((robot.DESIRED_TAG_ID < 0 || detection.id == 1)) {
                                //If sees Blue Left
                                if (robot.DESIRED_TAG_ID > 1) {
                                    robot.mecanumDrive(-0.5, 0, 0);
                                    //if Spike indicated Ceter or Right
                                }
                            } else if ((robot.DESIRED_TAG_ID < 0 || detection.id == 2)) {
                                //If sees Blue Center
                                if (robot.DESIRED_TAG_ID > 2) {
                                    robot.mecanumDrive(-0.5, 0, 0);
                                    //If spike indicated Right
                                } else if (robot.DESIRED_TAG_ID < 2) {
                                    robot.mecanumDrive(0.5, 0, 0);
                                    //if spike indicated Left
                                }
                            } else if ((robot.DESIRED_TAG_ID < 0 || detection.id == 3)) {
                                //If sees Blue Right
                                 if (robot.DESIRED_TAG_ID < 3) {
                                    robot.mecanumDrive(0.5, 0, 0);
                                    //If spike indicated Center or Left
                                }
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        }
                    }
                    if (targetFound) {
                        telemetry.addLine("\nFound!");
                        telemetry.addData("\n", robot.DESIRED_TAG_ID);
                        robot.mecanumDrive(0, 0, 0);
                        //sends a the robot sees the correct tag
                    } else {
                        telemetry.addLine("\nNot found.");
                    }
                    robot.telemetryAprilTag();
                    // Save CPU resources; can resume streaming when needed.
                    // if (gamepad1.dpad_down) {
                    //    visionPortal.stopStreaming();
                    ///  } else if (gamepad1.dpad_up) {
                    //   visionPortal.resumeStreaming();
                    //      }
                    telemetry.update();
                    // Share the CPU.
                    sleep(20);
                }
            }
            // Save more CPU resources when camera is no longer needed.
            robot.visionPortal.close();
        }
    }
}