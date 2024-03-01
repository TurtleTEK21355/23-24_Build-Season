
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@Autonomous(name="BlueBackAuto_Vision", group="Red Vision Team")
public class BlueBackVision extends LinearOpMode {
    RobotHardware_TT robot = new RobotHardware_TT(this);

    //19.2 * 28 = 96πmm <-- Replace these numbers.
    //537.6 ticks = 301.6mm
    // 1 tick = 0.561mm
    double tickToMMRatio = 0.561 / 1;
    int startEncoderValue;

    String Tag = "Unseen";

    CenterstageDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {


        robot.init();
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new CenterstageDeterminationPipeline();
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
        long timer;
        waitForStart();
        sleep(1000);
        telemetry.addData("Y Region 1", pipeline.getAvgYRegion1());
        telemetry.addData("Y Region 2", pipeline.getAvgYRegion2());
        telemetry.addData("The Region", pipeline.WhichRegion());
        telemetry.update();
        robot.autoDrive(20, 0.2);
        robot.autoStrafe(150,-0.2);
        if (pipeline.WhichRegion() == 1) {
            telemetry.addLine("Left");
            telemetry.update();
            robot.autoDrive(775, 0.2); //don't know true numbers
            robot.autoTurn(90, 0.2);
            robot.setIntake(0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +2000){}
            robot.setIntake(0);
            robot.autoDrive(50,-0.2);
            robot.resetImu();
            robot.autoTurn(-90, 0.2);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(750, -0.3);
            robot.autoDrive(1100,0.3);
            robot.autoStrafe(-3800,0.5);
            robot.autoDrive(200,-0.5);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        }
        if (pipeline.WhichRegion() == 2) {
            telemetry.addLine("Center");
            telemetry.update();
            robot.autoDrive(890, 0.2);
            robot.setIntake(0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 2000) {
            }
            robot.setIntake(0);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(450, -0.2);
            robot.autoDrive(1050,0.3);
            robot.autoStrafe(-3650,0.5);
            robot.autoDrive(300,-0.2);

            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {

            }
        } else {
            telemetry.addLine("Right");
            telemetry.update();
            robot.autoDrive(725, 0.2); //don't know true numbers
            robot.autoTurn(-90, 0.2);
            robot.autoDrive(100,0.2);
            robot.setIntake(0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer + 1000){}
            robot.setIntake(0);
            robot.autoDrive(-200,-0.2);
            robot.autoTurn(90, 0.2);
            robot.autoDrive(-150, -0.2);
            robot.autoStrafe(450, -0.2);
            robot.autoDrive(1050,0.3);
            robot.autoStrafe(-3650,0.5);
            robot.autoDrive(200,-0.2);
            timer = robot.eleapsedTime();
            while (opModeIsActive() && robot.eleapsedTime() < timer +20000) {}
        }
    }
    public static class CenterstageDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the Centerstage position
         */
        public enum PropPosition {
            LEFT,
            CENTER,
            RIGHT;

        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);


        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(140, 200);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(470, 170);
        static final int REGION_WIDTH = 125;
        static final int REGION_HEIGHT = 125;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        /*
         * Working variables
         */
        Mat region1, region2, region3;
        Mat imgYCrCb = new Mat();
        Mat maskedYCrCb = new Mat();
        Mat maskedRGB = new Mat();
        Mat Cb = new Mat();

        Mat Cr = new Mat();

        Mat Y = new Mat();
        volatile double avgYRegion1;
        volatile double avgYRegion2;
        volatile Scalar avgRegion1;


        int avgY;

        int avgCr;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile PropPosition position = PropPosition.LEFT;

        public PropPosition getPropPosition() {
            return position;
        }

        public double WhichRegion() {
            double ThisRegion = 0;
            if (avgYRegion1 > avgYRegion2) {
                ThisRegion = 1;
            } else if (avgYRegion2 > avgYRegion1) {
                ThisRegion = 2;
            } else {
                ThisRegion = 3;
            }
            return ThisRegion;
        }

        public double getAvgYRegion1() {
            return avgYRegion1;
        }

        public double getAvgYRegion2() {
            return avgYRegion2;
        }

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToYCrCb(Mat input) {
            Imgproc.cvtColor(input, imgYCrCb, Imgproc.COLOR_RGB2YCrCb);
        }

        @Override

        public void init(Mat firstFrame) {

            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToYCrCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
        }


        @Override
        public Mat processFrame(Mat input) {

            inputToYCrCb(input);
            Core.inRange(imgYCrCb, new Scalar(45,55,130), new Scalar(165,150,210), maskedYCrCb);
            region1 = maskedYCrCb.submat(new Rect(region1_pointA, region1_pointB));
            region2 = maskedYCrCb.submat(new Rect(region2_pointA, region2_pointB));

            avgYRegion1 = Core.mean(region1).val[0];
            avgYRegion2 = Core.mean(region2).val[0];
            avgRegion1 = Core.mean(region1);

            double max = Math.max(avgYRegion1, avgYRegion2);

            return maskedYCrCb;
        }


        public Scalar getColorRegion1() {
            return new Scalar(avgY, avgCr, avgYRegion1);
        }
    }
}
