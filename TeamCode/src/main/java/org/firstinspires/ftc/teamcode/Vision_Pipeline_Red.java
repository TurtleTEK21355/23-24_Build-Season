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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(140,200);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(470,170);
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
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToYCrCb(Mat input)
        {
            Imgproc.cvtColor(input, imgYCrCb, Imgproc.COLOR_RGB2YCrCb);
        }

        @Override

        public void init(Mat firstFrame)
        {

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
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToYCrCb(input);
            Core.inRange(imgYCrCb, new Scalar(50, 140, 55), new Scalar(175, 250, 150), maskedYCrCb);
            region1 = maskedYCrCb.submat(new Rect(region1_pointA, region1_pointB));
            region2 = maskedYCrCb.submat(new Rect(region2_pointA, region2_pointB));
            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */


            /*
             * Find the max of the 3 averages
             */
            avgYRegion1 = Core.mean(region1).val[0];
            avgYRegion2 = Core.mean(region2).val[0];
            double max = Math.max(avgYRegion1, avgYRegion2);


            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if (max == avgYRegion1) // Was it from region 1?
            {
                position = PropPosition.LEFT; // Record our analysis


                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        maskedYCrCb, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill


            } else if (max == avgYRegion2) // Was it from region 2?
            {
                position = PropPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        maskedYCrCb, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }


            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return maskedYCrCb;
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
