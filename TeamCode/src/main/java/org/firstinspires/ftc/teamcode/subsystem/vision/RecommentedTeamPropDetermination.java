/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.subsystem.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the teamProp when lined up with
 * the sample regions over the 3 possible locations.
 */
@Disabled
@TeleOp
public class RecommentedTeamPropDetermination extends LinearOpMode
{
    public static int leftRegionAvgCr;
    public static int centerRegionAvgCr;
    public static int rightRegionAvgCr;
    OpenCvWebcam webCam;
    TeamPropDeterminationPipeline pipeline;
    //    webcam.setPipeline(new pipeline());

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TeamPropDeterminationPipeline();
        webCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
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
            //webCam.startStreaming(640,480);
            telemetry.addData("Analysis", pipeline.getAnalysis());
            //telemetry.addData("Pixel Cr Average = ", String.valueOf(leftRegionAvgCr), String.valueOf(centerRegionAvgCr), String.valueOf(rightRegionAvgCr));
            telemetry.addData("Left Region cr", leftRegionAvgCr);
            telemetry.addData("Center Region cr", centerRegionAvgCr);
            telemetry.addData("Right Region cr", rightRegionAvgCr);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
        }
    }

    public static class TeamPropDeterminationPipeline extends OpenCvPipeline
    {
        public static char detectColor = 'R';
        /*
         * An enum to define the TeamProp position
         */
        public enum TeamPropPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point leftSpikeTopLeftPoint = new Point(370,900);
        static final Point middleSpikeTopLeftPoint = new Point(450,420);
        static final Point rightSpikeTopLeftPoint = new Point(370,50);
        static final int detectionBoxWidth = 350;
        static final int detectionBoxHeight = 250;

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
        Point leftSpikeTopLeft = new Point(
                leftSpikeTopLeftPoint.x,
                leftSpikeTopLeftPoint.y);
        Point leftSpikeBottomRight = new Point(
                leftSpikeTopLeftPoint.x + detectionBoxWidth,
                leftSpikeTopLeftPoint.y + detectionBoxHeight);
        Point middleSpikeTopLeft = new Point(
                middleSpikeTopLeftPoint.x,
                middleSpikeTopLeftPoint.y);
        Point middleSpikeBottomRight = new Point(
                middleSpikeTopLeftPoint.x + detectionBoxHeight,
                middleSpikeTopLeftPoint.y + detectionBoxWidth);
        Point rightSpikeTopLeft = new Point(
                rightSpikeTopLeftPoint.x,
                rightSpikeTopLeftPoint.y);
        Point rightSpikeBottomRight = new Point(
                rightSpikeTopLeftPoint.x + detectionBoxWidth,
                rightSpikeTopLeftPoint.y + detectionBoxHeight);

        /*
         * Working variables
         */
        Mat region1_Cr, region2_Cr, region3_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        /*Mat HSV = new Mat();
        Mat RedLow = new Mat();
        Mat RedHigh = new Mat();
        Mat RedPixels = new Mat();
        Scalar RedLowLowerBound = new Scalar(0, 255, 255);// set lower bound for HSV values for low range of Red
        Scalar RedLowUpperBound = new Scalar(10, 255, 255); // set upper bound for HSV values for low range Red
        Scalar RedHighLowerBound = new Scalar(160, 200, 30);// set lower bound for HSV values for high range of Red
        Scalar RedHighUpperBound = new Scalar(180, 255, 255);// set upper bound for HSV values for high range of Red
        */ //HSV code stuff

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile TeamPropPosition position = TeamPropPosition.RIGHT;
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cr channel to the 'Cr' variable
         */
        void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);// convert to YCrCb
            if (detectColor == 'r'||detectColor=='R'){
                Core.extractChannel(YCrCb, Cr, 1);// Extract only the Red (Cr) values
            }else{
                Core.extractChannel(YCrCb, Cr, 2);// Extract only the Blue (Cb) values
            }

/* added lines below to convert RGB image to HSV format. After that we use the Hue values to
fing only Red Pixels or only Blue pixels using inRange thresholding method in OpenCV. This will
help us find where the teamProp is on the field

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);// convert to HSV
            //Core.extractChannel(HSV, Hr, 0);// Extract only the Hue values
            //Get Red pixels with Hue between 0 and 10
            Core.inRange(Hr, RedLowLowerBound, RedLowUpperBound, RedLow);
            //Get Red pixels with Hue between 160 and 180
            Core.inRange(HSV, RedHighLowerBound, RedHighUpperBound, RedHigh);
            // Combine the binary images to get all red pixels
            Core.add(RedLow, RedHigh, RedPixels); // the Mat "RedPixels" includes all red pixels in the image
        */
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cr'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCr(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cr = Cr.submat(new Rect(leftSpikeTopLeft, leftSpikeBottomRight));
            region2_Cr = Cr.submat(new Rect(middleSpikeTopLeft, middleSpikeBottomRight));
            region3_Cr = Cr.submat(new Rect(rightSpikeTopLeft, rightSpikeBottomRight));
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
             *
             * After we've converted to YCrCb, we extract just the 1st channel, the
             * Cr channel. We do this because teamProp is red and we want to find if
             * the color in the region of interest is red.
             *
             * We then take the average pixel value of 3 different regions on that Cr
             * channel, one positioned over each region of interest. The dimmest
             *  of the 3 regions is where we assume the teamProp to be.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the teamProp.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the three possible locations,
             * and be small enough such that only the center of the position is sampled,
             * and not any of the surroundings.
             */
            /*
             * Get the Cr channel of the input frame after conversion to YCrCb
             */
            inputToCr(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */

//            getAverage(leftRegionAvgCr, region1_Cr);
//            getAverage(centerRegionAvgCr, region2_Cr);
//            getAverage(rightRegionAvgCr, region3_Cr);
            leftRegionAvgCr = (int) Core.mean(region1_Cr).val[0];
            centerRegionAvgCr = (int) Core.mean(region2_Cr).val[0];
            rightRegionAvgCr = (int) Core.mean(region3_Cr).val[0];
            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    leftSpikeTopLeft, // First point which defines the rectangle
                    leftSpikeBottomRight, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    middleSpikeTopLeft, // First point which defines the rectangle
                    middleSpikeBottomRight, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    rightSpikeTopLeft, // First point which defines the rectangle
                    rightSpikeBottomRight, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


             /*
             * Find the min of the 3 averages
             */
            int maxOneTwo = Math.max(leftRegionAvgCr, centerRegionAvgCr);
            int max = Math.max(maxOneTwo, rightRegionAvgCr);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == leftRegionAvgCr) // Was it from region 1?
            {
                position = TeamPropPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        leftSpikeTopLeft, // First point which defines the rectangle
                        leftSpikeBottomRight, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == centerRegionAvgCr) // Was it from region 2?
            {
                position = TeamPropPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        middleSpikeTopLeft, // First point which defines the rectangle
                        middleSpikeBottomRight, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == rightRegionAvgCr) // Was it from region 3?
            {
                position = TeamPropPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        rightSpikeTopLeft, // First point which defines the rectangle
                        rightSpikeBottomRight, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;

        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public TeamPropPosition getAnalysis() {
            return position;
        }
                                   public void getAverage (int avg,Mat regionCr){
            avg = (int) Core.mean(regionCr).val[0];
        }
    }
}