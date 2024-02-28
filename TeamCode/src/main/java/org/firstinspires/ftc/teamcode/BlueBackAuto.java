package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "BlueBackAuto_Vision", group = "Blue Vision Team")
public class BlueBackAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm
    double tickToMMRatio = 0.561 / 1;
    int startEncoderValue;


    CenterstageDeterminationPipelineBlue pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData(">", cameraMonitorViewId);
        telemetry.update();
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CenterstageDeterminationPipelineBlue();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        boolean targetFound = false;


        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        waitForStart();

        long timer = 0;
        //Seeing stuff here
        sleep(100);
        robot.autoDrive(100, 0.2);
        if (pipeline.WhichRegion() == 1) {
            //Left
            // Here is where you would put code to place the pixel on the spike mark
            telemetry.addLine("Left");
            telemetry.update();
            robot.autoDrive(725, 0.25);
            robot.autoTurn(90, 0.3);
            robot.autoDrive(50, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 2000) {
            }
            robot.setIntake(0);
            robot.autoDrive(80, -0.2);
            robot.autoTurn(-90, 0.2);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(-450, -0.3);
            robot.autoDrive(950, 0.3);
            robot.autoStrafe(3750, 0.5);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 20000) {
            }
        } else if (pipeline.WhichRegion() == 2) {
            //Center
            // Here is where code to place the pixel on the spike mark is.
            robot.autoDrive(725, 0.2);
            robot.autoStrafe(50, -0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 2000) {
            }
            robot.setIntake(0);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(450, -0.2);
            robot.autoDrive(950, 0.3);
            robot.autoStrafe(3650, 0.5);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 20000) {
            }


        } else {
            //Right
            // Here is where you would put code to place the pixel on the spike mark
            robot.autoDrive(725, 0.2);
            robot.autoTurn(-90, 0.3);
            robot.autoDrive(50, 0.2);
            robot.setIntake(-0.25);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 1000) {
            }
            robot.setIntake(0);
            robot.autoDrive(-150, -0.2);
            robot.autoTurn(90, 0.2);
            robot.autoDrive(-250, -0.2);
            robot.autoStrafe(400, -0.2);
            robot.autoDrive(950, 0.3);
            robot.autoStrafe(3650, 0.5);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 20000) {
            }
        }

        robot.autoDrive(-1650, 0.2);
        robot.autoStrafe(3350, 0.2);
    }
}