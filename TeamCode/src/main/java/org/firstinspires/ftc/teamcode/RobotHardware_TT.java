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


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

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
    // private DcMotor leftDrive;
    // private DcMotor rightDrive;
    private Servo claw1;
    private Servo wrist;
    private DcMotor launch;
    private DcMotor intakeMotor;
    private DcMotor armMotor;
    private IMU scootImu;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private DcMotor pixelMotor;
    //  private Servo claw1;
    //private Servo claw2;
    private DigitalChannel touchSensor;
    // private double pastEncoder = Double.NEGATIVE_INFINITY;
    //private static final String VUFORIA_KEY = LicenseKey.key;
//    private VuforiaLocalizer vuforia;


  /*  public TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    public static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    */


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
/*
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *left
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        //leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "motorLeft");
        //rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "motorRight");

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
        launch = myOpMode.hardwareMap.get(DcMotor.class, "launch");
        armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "motorArm");
        intakeMotor   = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        //armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "motorArm");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        //leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);
        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //claw1 = myOpMode.hardwareMap.get(Servo.class, "claw1");
        //claw2 = myOpMode.hardwareMap.get(Servo.class, "claw2");


        scootImu = myOpMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        scootImu.initialize(new IMU.Parameters(orientationOnRobot));
        scootImu.resetYaw();
        //RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        //RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        //RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //imu.initialize(new IMU.Parameters(orientationOnRobot));


        touchSensor = myOpMode.hardwareMap.get(DigitalChannel.class,"touchSensor");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void initAuto() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "motorLeft");
        //rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "motorRight");
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
/*        //armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "motorArm");
        //imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        //leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
 //       rightDrive.setDirection(DcMotor.Direction.FORWARD);
   //     armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
     //   armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        //claw1.setPosition(0);
        //claw2.setPosition(1);

        //touchSensor = myOpMode.hardwareMap.get(DigitalChannel.class,"touchSensor");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    *//**
         * Calculates the left/right motor powers required to achieve the requested
         * robot motions: Drive (Axial motion) and Turn (Yaw motion).
         * Then sends these power levels to the motors.
         *
         * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
         *//*
    public void arcadeDrive(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive - Turn;
        double right = Drive + Turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        tankDrive(left, right);
    }

    *//**
         * Pass the requested wheel motor powers to the appropriate hardware drive motors.
         *
         * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         *//*
    public void tankDrive(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        //leftDrive.setPower(-leftWheel);
        //rightDrive.setPower(-rightWheel);
    }*/
    }

    public void mecanumDrive(double x, double y, double heading) {
        double newRx;
        newRx = turnValue(heading);


//        leftFrontDrive.setPower(y + x + newRx);
//        rightFrontDrive.setPower(y + x - newRx);
//        leftBackDrive.setPower(y - x + newRx);
//        rightBackDrive.setPower(y - x - newRx);
        leftFrontDrive.setPower(newRx + x + y);
        rightFrontDrive.setPower(newRx + x - y);
        leftBackDrive.setPower(newRx - x + y);
        rightBackDrive.setPower(newRx - x - y);
    }

    public List<Integer> getEncoders(){
        List<Integer> encoderValues = new ArrayList<Integer>();
        encoderValues.add(leftFrontDrive.getCurrentPosition());
        encoderValues.add(rightFrontDrive.getCurrentPosition());
        encoderValues.add(leftBackDrive.getCurrentPosition());
        encoderValues.add(rightBackDrive.getCurrentPosition());
        return encoderValues;
    }

    public void resetEncoders(){
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    

    public double imuTurn(double turnToAngle) {
        double correctionRx;
        double toleranceValue = 7.5;
        //Taking an IMU reading
        YawPitchRollAngles orientation = scootImu.getRobotYawPitchRollAngles();
        double yawDegrees = orientation.getYaw(AngleUnit.DEGREES);
        // Adjust the correctionRx value by that result * -1
        double correctionYawDegrees = yawDegrees * -1;
        // if correctionYawDegrees < 0 then turn left.
        if (correctionYawDegrees > turnToAngle + 1) {
            correctionRx = 1;
        } else if (correctionYawDegrees < turnToAngle - toleranceValue) {
            correctionRx = -1;
        } else {
            correctionRx = 0;
        }

        return correctionRx;
        //correctionRx = correctionYawDegrees/180;
        // return correctionRx;

        // if correctionYawDegrees > 0 then turn right.
        // else, do nothing.
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

    public void setClaw1(double claw) {
        if (claw > 0.20){
            claw1.setPosition(22);
        }
        else {
            claw1.setPosition(claw);
        }
    }
    public void setWrist(double wrist1) {
        wrist.setPosition(wrist1);
    }
    public void setLaunch(double power) {
        launch.setPower(power);
    }
    public void setArm (double arm) {
        armMotor.setPower(arm);
    }
    public void setIntake (double speed) {
        intakeMotor.setPower(speed);
    }
      public boolean touchSensorNotPressed(){
          return touchSensor.getState();
      }
       public boolean touchSensorIsPressed(){
         return !touchSensor.getState();
      }
    public void capturePixel () {
        if (touchSensorIsPressed()) {
            claw1.setPosition(22);
        } else if (touchSensorNotPressed()) {
            myOpMode.telemetry.addLine("\nTouch Sensor not detecting Pixel.");
        }
    }
    public void driveMecanum();

    }

}
/*


   */
/* public void getAngle(){
        Orientation heading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }*//*

    */
/**
     * The robot eats pizza. Not really. It drives for a distance of x feet.
     *
     *//*

    public double getWheelEncoderValue() {
       // return leftDrive.getCurrentPosition();
    //}
    //public double getWheelInches() {
        return getWheelEncoderValue() * .0219;
    }
    public void driveDistance(double feet) {
       // rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myOpMode.sleep(100);
        double inches = (feet * 12)*-1;
        //myOpMode.telemetry.addData("inches : ", inches);
        //myOpMode.telemetry.update();
        myOpMode.sleep(100);
        tankDrive(.35, .35);
        //while (getWheelInches() > inches && myOpMode.opModeIsActive()) {
            tankDrive(.35, .35);
          //  armHeight(2);
          //  setHandPosition(2.04,-1.04);
        }
     //   tankDrive(0,0);
      //  myOpMode.sleep(100);
   //}

    //private double getPastEncoder() {
        String Encoder;
       // if (pastEncoder == Double.NEGATIVE_INFINITY){
            String filename = "AutoEncoder.txt";
            File file = AppUtil.getInstance().getSettingsFile(filename);
           // Encoder = ReadWriteFile.readFile(file);
         //   pastEncoder = Double.parseDouble(Encoder);
           // myOpMode.telemetry.addData("Encoder value: ", Encoder);
       // }
       // return pastEncoder;
    //}


//    public double getArmEncoderValue() {
      //  return armMotor.getCurrentPosition() + getPastEncoder();
   // }
   // public double getArmInches() {
   //     return getArmEncoderValue() * .0082;
    //}

    */
/**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     *//*

    public void setArmPower(double power) {
    //    if (getArmEncoderValue() <=  10 && power < 0) {
        //    armMotor.setPower(0);
      //  } else{
          //  armMotor.setPower(power);
        }
  //  }

    */
/*public void armHeight(double height) {
        if (getArmInches()  < height) {
            setArmPower(0.8);
        } else {
            setArmPower(0);
        }
    }
*/
/* //public boolean touchSensorNotPressed(){
        return touchSensor.getState();
    //}
    //public boolean touchSensorIsPressed(){
       // return !touchSensor.getState();
    //}

/*
    //public void TurnLeft() {
        //imu.resetYaw();
   //     tankDrive(-0.75, 0.75);
       // while (getAngle() < 85 && myOpMode.opModeIsActive()) {
            tankDrive(-0.75, 0.75);
          //  armHeight(2);
        //    myOpMode.telemetry.addData("", getAngle());
            myOpMode.telemetry.update();
        //}
      //  tankDrive(0,0);
   // }
    //public void TurnRight() {
       // imu.resetYaw();
        tankDrive(0.75, -0.75);
      //  while (getAngle() > -85 && myOpMode.opModeIsActive()) {
            tankDrive(0.75, -0.75);
         //   armHeight(2);
         //   myOpMode.telemetry.addData("", getAngle());
            myOpMode.telemetry.update();
        //}
        //tankDrive(0,0);
    //}

    //public void wake(long milliseconds) {
        ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
     //   while (milliseconds > elapsedTime.time()) {
           // armHeight(2);
        }
    }
}

*/
