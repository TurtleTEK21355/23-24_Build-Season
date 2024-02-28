
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name="RedFrontAuto_Vision", group="Red Vision Team")
public class RedFrontAuto extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm
    double tickToMMRatio = 0.561 / 1;
    int startEncoderValue;

    String Tag = "Unseen";

    CenterstageDeterminationPipelineRed pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData(">", cameraMonitorViewId);
        telemetry.update();
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CenterstageDeterminationPipelineRed();
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
        robot.resetImu();

        List<Integer> encoderList = robot.getEncoders();
        startEncoderValue = encoderList.get(0);
        long timer = 0;
        waitForStart();
// seeing stuff here

        sleep(100);
        robot.autoDrive(100, 0.2);
        if (pipeline.WhichRegion() == 1) {
            //Left
            // Here is where you would put code to place the pixel on the spike mark
            telemetry.addLine("Left");
            telemetry.update();
            robot.autoDrive(725, 0.2); //don't know true numbers
            robot.autoTurn(90,0.2);
            robot.autoDrive(150, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(-150,-0.2);
            robot.autoTurn(-90, 0.2);
            robot.autoDrive(-600,-0.2);
            robot.autoStrafe(-1200, -0.4); //don't know true numbers
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        } else if (pipeline.WhichRegion() == 2) {
            //Center
            // Here is where code to place the pixel on the spike mark is.
            telemetry.addLine("Center");
            telemetry.update();
            robot.autoDrive(725, 0.2);
            robot.autoStrafe(50, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoStrafe(-200, -0.4);
            robot.autoDrive(-600, -0.2);
            robot.autoStrafe(-1200, -0.4);
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        } else {
            //Right
            // Here is where you would put code to place the pixel on the spike mark
            //need numbers
            telemetry.addLine("Right");
            telemetry.update();
            robot.autoDrive(725, 0.2); //don't know true numbers
            robot.autoTurn(-90,0.2);
            robot.autoDrive(150, 0.2);
            robot.setIntake(-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +1000){}
            robot.setIntake(0);
            robot.autoDrive(-170,-0.2);
            robot.autoTurn(90, 0.2);
            robot.autoStrafe(100, 0.2);
            robot.autoDrive(-675,-0.2);
            robot.autoStrafe(-1200, -0.4); //don't know true numbers
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        }
    }
}