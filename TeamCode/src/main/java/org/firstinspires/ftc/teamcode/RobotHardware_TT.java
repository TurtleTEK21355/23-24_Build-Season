/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class RobotHardware_TT {
    //Base Code

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)

    public Servo claw1;
    public Servo claw2;
    private Servo wrist;
    private Servo flick;
    private DcMotor launch;
    private DcMotor intakeMotor;
    private DcMotor armMotor;
    private IMU scootImu;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public int DESIRED_TAG_ID = 6;   // Choose the tag you want to approach or set to -1 for ANY tag.
    public AprilTagDetection desiredTag = null;
    private DcMotor pixelMotor;
    public final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    /*  IMU imu;
      // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
      public static final double MID_SERVO       =  0.5 ;
      public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
      public static final double ARM_UP_POWER    =  0.45 ;
      public static final double ARM_DOWN_POWER  = -0.45 ;
  */
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware_TT(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *left
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
        launch = myOpMode.hardwareMap.get(DcMotor.class, "launch");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "motorArm");
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        huskyLens = myOpMode.hardwareMap.get(HuskyLens.class, "huskylens");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Define and initialize ALL installed servos.
        claw1 = myOpMode.hardwareMap.get(Servo.class, "claw1");
        claw2 = myOpMode.hardwareMap.get(Servo.class, "claw2");
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        flick = myOpMode.hardwareMap.get(Servo.class, "linear");
        flick.setPosition(0.8);



        scootImu = myOpMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        scootImu.initialize(new IMU.Parameters(orientationOnRobot));
        scootImu.resetYaw();



        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void initAuto() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
/*        //armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "motorArm");
        //imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //imu.initialize(new IMU.Parameters(orientationOnRobot));

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //claw1 = myOpMode.hardwareMap.get(Servo.class, "claw1");
        //claw2 = myOpMode.hardwareMap.get(Servo.class, "claw2");



        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();*/
    }


    public void mecanumDrive(double x, double y, double heading) {
        double newRx = 0;
        myOpMode.telemetry.addData("New RX value", newRx);
          y*=-1;
          newRx = turnValue(-heading);


//        leftFrontDrive.setPower(y + x + newRx);
//        rightFrontDrive.setPower(y + x - newRx);
//        leftBackDrive.setPower(y - x + newRx);
//        rightBackDrive.setPower(y - x - newRx);
        leftFrontDrive.setPower(newRx + x + y);
        rightFrontDrive.setPower(newRx + x - y);
        leftBackDrive.setPower(newRx - x + y);
        rightBackDrive.setPower(newRx - x - y);
    }

    public List<Integer> getEncoders() {
        List<Integer> encoderValues = new ArrayList<Integer>();
        encoderValues.add(leftFrontDrive.getCurrentPosition());
        encoderValues.add(rightFrontDrive.getCurrentPosition());
        encoderValues.add(leftBackDrive.getCurrentPosition());
        encoderValues.add(rightBackDrive.getCurrentPosition());
        return encoderValues;
    }


    public void resetEncoders() {
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runLeftFrontMotor(){
        leftFrontDrive.setPower(0.2);
    }
    public void runRightBackMotor(){
        rightBackDrive.setPower(0.2);
    }
    public void runRightFrontMotor(){
        rightFrontDrive.setPower(0.2);
    }
    public void runLeftBackMotor(){
        leftBackDrive.setPower(0.2);
    }


    public double imuTurn(double turnToAngle) {
        double correctionRx;
        double toleranceValue = 7.5;
        //Taking an IMU reading
        YawPitchRollAngles orientation = scootImu.getRobotYawPitchRollAngles();
        double yawDegrees = orientation.getYaw(AngleUnit.DEGREES);
        // Adjust the correctionRx value by that result * -1
        double correctionYawDegrees = -yawDegrees * yawDegrees;
        // if correctionYawDegrees < 0 then turn left.
        if (correctionYawDegrees > turnToAngle + toleranceValue) {
            correctionRx = 1;
        } else if (correctionYawDegrees < turnToAngle - toleranceValue) {
            correctionRx = -1;
        } else {
            correctionRx = 0;
        }

        return correctionRx;
//            correctionRx = correctionYawDegrees/180;
//            //return correctionRx;

        // if correctionYawDegrees > 0 then turn right.
        // else, do nothing.
    }

    double getYawAngles() {
        double yawAngle = scootImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return yawAngle;
    }

    double turnValue(double desiredAngle) {
        YawPitchRollAngles orientation = scootImu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        //The angle based on where you where facing when you started.
        double correctionAngle = desiredAngle - currentAngle;
        double tValue = 0;
        //When I want to turn right, tValue should be positive.
        // When I want to turn left, tValue should be negative.
        if (correctionAngle > 180) {
            //turn left
            tValue = -0.6;
        } else if (correctionAngle > 4) {
            //turn right
            tValue = 0.6;
        }

        if (correctionAngle < -180) {
            //turn right
            tValue = 0.6;
        } else if (correctionAngle < -4) {
            //turn left
            tValue = -0.6;
        }
        return tValue;
    }


    double pValue = -1;
    double elevatorPValue = -1;

    double porportionalController(double input, double goalReading) {
        double arbitraryValue = (goalReading - input) * pValue;
        return arbitraryValue;
    }


    double porportionalElevatorControl(double goalHeight, double encoderPosition) {
        double motorPosition = (goalHeight - encoderPosition) * elevatorPValue;
        return motorPosition;
    }

    double armMotorEncoders() {
        double armMotorPosition = armMotor.getCurrentPosition();
        return armMotorPosition;
    }

    /**
     * @param clawOne Sets left claw position. Be careful with the number restraints!
     *
     */
    public void setClaw(double clawOne) {
        //need to work on exact numbers
        double clawTwo;
        if (clawOne > 0.20) {
            claw1.setPosition(22);
            clawTwo = 1-clawOne;
        } else {
            claw1.setPosition(clawOne);
            clawTwo = 1-clawOne;
        }
        if (clawTwo < 0.80) {
            claw2.setPosition(78);
        } else {
            claw2.setPosition(clawTwo);
        }
    }

    /**
     * @param wrist1 sets wrist position
     */
    public void setWrist(double wrist1) {
        wrist.setPosition(wrist1);
    }

    /**
     * @param flick1 linear Servo; max: 0.8 min: 0.2
     */
    public void linearServoGo(double flick1) {
        flick.setPosition(flick1);
    }

    /**
     * @param power spins wheel for Drone launch. Do NOT go over 0.8
     */
    public void setLaunch(double power) {
        launch.setPower(power);
    }

/**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param arm driving power (-1.0 to 1.0)
     */
    public void setArm(double arm) {
        armMotor.setPower(arm);
    }

    /**
     * @param speed Sets 3D printed intake speed. Do NOT go over 0.7
     */
    public void setIntake(double speed) {
        intakeMotor.setPower(speed);
    }

    public void initLens() {
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            myOpMode.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            myOpMode.telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        myOpMode.telemetry.update();
    }
    /**
     * If you use this also use blockLensY.
     * This is for the x-value with the HuskyLens.
     * */
    public int blockLensX() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        myOpMode.telemetry.addData("Block count", blocks.length);
        int x = 0;
        for (int i = 0; i < blocks.length; i++) {
            x = blocks[i].x;
            myOpMode.telemetry.addData("\nX:", blocks[i].x);
            myOpMode.telemetry.addData("Blocks: ", blocks[i].toString());
        }

        myOpMode.telemetry.update();
        return x;
    }
    /**
     * If you use this also use blockLensX.
     * This is for the y-value with the HuskyLens.
     * */
    public int blockLensY() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        myOpMode.telemetry.addData("Block count", blocks.length);
        int y = 0;
        for (int i = 0; i < blocks.length; i++) {
            y = blocks[i].y;
            myOpMode.telemetry.addData("\nY:", blocks[i].y);
            myOpMode.telemetry.addData("Blocks: ", blocks[i].toString());
        }

        myOpMode.telemetry.update();
        return y;
    }



    /**
     * This is for
     *
     * @param movementSpeed  a variable betwwen 0.0 and +1.0
     * @param strafeDistance inches, lateral movement
     * @param driveDistance  inches, standard forward/reverse movement
     */
    public void driveMecanum(double driveDistance, double strafeDistance, double movementSpeed) {
        /*Reads all of the encoder values, and causes the motors to run, whether in a strafing or
        standard movement, as long as the encoder value is less than the desired distance value.
         */
    }

    public void initAprilTag() {

            // Create the AprilTag processor the easy way.
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();

            // Create the vision portal the easy way.


        }

        /**
         * Add telemetry about AprilTag detections.
         */
        public void telemetryAprilTag() {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            myOpMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            myOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            myOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            myOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");

        }



}