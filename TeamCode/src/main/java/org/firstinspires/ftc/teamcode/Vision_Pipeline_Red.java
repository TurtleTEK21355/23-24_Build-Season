package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


@TeleOp(name = "VisionPipelineRed", group="Purzple Team")

public class Vision_Pipeline_Red extends LinearOpMode{
    RobotHardware_Calibration robot;
    CenterstageDeterminationPipeline pipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware_Calibration(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData(">", cameraMonitorViewId);
        telemetry.update();
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CenterstageDeterminationPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();

        while (opModeIsActive())
        {
            //telemetry.addData("Analysis", pipeline.getAnalysis());
            //telemetry.update();
            pipeline.getAnalysis();
            telemetry.addData("Which Square?", pipeline.WhichRegion());
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            telemetry.update();
        }

    }
    public static class CenterstageDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the Centerstage position
         */
        public enum PropPosition
        {
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(120,200);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(515,98);
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
        double avgYRegion1;
        double avgYRegion2;


        int avgY;

        int avgCr;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile PropPosition position = PropPosition.LEFT;

        public PropPosition getPropPosition() {
            return position;
        }

        public double WhichRegion(){
            double ThisRegion = 0;
            if (avgYRegion1 > avgYRegion2){
                ThisRegion = 1;
            }
            else if (avgYRegion2 > avgYRegion1){
                ThisRegion = 2;
            }
            return ThisRegion;
        }

        void inputToYCrCb(Mat input)
        {
            Imgproc.cvtColor(input, imgYCrCb, Imgproc.COLOR_RGB2YCrCb);
        }

        @Override

        public void init(Mat firstFrame)
        {

            inputToYCrCb(firstFrame);

        }


        @Override
        public Mat processFrame(Mat input)
        {

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public PropPosition getAnalysis()
        {
            return position;
        }
        public Scalar getColorRegion1()
        {
            return new Scalar(avgY,avgCr, avgYRegion1);
        }
    }
}
