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
@TeleOp
public class HSVPropDetermination extends LinearOpMode{
    public static int leftRegionAvgHSV;
    public static int centerRegionAvgHSV;
    public static int rightRegionAvgHSV;
    public static int offset = 100;
    OpenCvWebcam webCam;
    HSVPropDeterminationPipeline pipeline;
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
        pipeline = new HSVPropDeterminationPipeline();
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
                //webCam.startStreaming(1920,1080, OpenCvCameraRotation.SIDEWAYS_LEFT);

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
            telemetry.addData("Left Region cr", leftRegionAvgHSV);
            telemetry.addData("Center Region cr", centerRegionAvgHSV);
            telemetry.addData("Right Region cr", rightRegionAvgHSV);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(100);
//            for (int i = 0; i < HSVTeamElementDetector.TeamPropDeterminationPipeline.HSV.rows(); i++) {
//                System.out.println(Arrays.toString(HSV.get(i, 0))); // get the row at i and convert it to a string and print it
//            }
//            sleep(10000);
        }
    }

    public static class HSVPropDeterminationPipeline extends OpenCvPipeline
    {
        int lineThickness = 4;
        public static char detectColor = 'r';
        /*
         * An enum to define the TeamProp position
         */
        public enum TeamPropPosition
        {LEFT, CENTER, RIGHT}

        /*
         * Some color constants
         */
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point leftSpikeTopLeftPoint = new Point(370, 900+offset);
        static final Point middleSpikeTopLeftPoint = new Point(350, 420+offset);
        static final Point rightSpikeTopLeftPoint = new Point(370, 50+offset);
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
        Mat region1_HSV, region2_HSV, region3_HSV;
        Mat HSV = new Mat();
        Mat RedLow = new Mat();
        Mat RedHigh = new Mat();
        Mat targetColorPixels = new Mat();
        Scalar RedLowLowerBound = new Scalar(0, 200, 30);// set lower bound for HSV values for low range of Red
        Scalar RedLowUpperBound = new Scalar(20, 255, 255); // set upper bound for HSV values for low range Red
        Scalar RedHighLowerBound = new Scalar(155, 200, 30);// set lower bound for HSV values for high range of Red
        Scalar RedHighUpperBound = new Scalar(180, 255, 255);// set upper bound for HSV values for high range of Red
        /*
        since red appears in two different locations in the HSV spectrum (beginning and the end), it must have a
        high and low bound.

         This is not the case with blue as it only shows up once
         */
        Scalar BlueLowerBound = new Scalar(90, 200, 30); //lower bound for blue
        Scalar BlueUpperBound = new Scalar(130, 255, 255); //upper bound for blue
        //HSV code stuff

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile TeamPropPosition position = TeamPropPosition.RIGHT;
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cr channel to the 'Cr' variable
         */
        void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);// convert to HSV
            if (detectColor == 'r'|| detectColor=='R'){
                Core.inRange(HSV, RedHighLowerBound, RedHighUpperBound, RedHigh);// Top Red Values
                Core.inRange(HSV, RedLowLowerBound, RedLowUpperBound, RedLow);// Lower Red Values
                Core.add(RedLow, RedHigh, targetColorPixels); //combines both sections of red pixels into one Mat
            }else{
                Core.inRange(HSV, BlueLowerBound, BlueUpperBound, targetColorPixels);// Blue Values
            }

/* added lines below to convert RGB image to HSV format. After that we use the Hue values to
fing only Red Pixels or only Blue pixels using inRange thresholding method in OpenCV. This will
help us find where the teamProp is on the field
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
            inputToHSV(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_HSV = targetColorPixels.submat(new Rect(leftSpikeTopLeft, leftSpikeBottomRight));
            region2_HSV = targetColorPixels.submat(new Rect(middleSpikeTopLeft, middleSpikeBottomRight));
            region3_HSV = targetColorPixels.submat(new Rect(rightSpikeTopLeft, rightSpikeBottomRight));
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
            inputToHSV(input);

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
            leftRegionAvgHSV = (int) Core.mean(region1_HSV).val[0];
            centerRegionAvgHSV = (int) (Core.mean(region2_HSV).val[0]* 1.5);
            rightRegionAvgHSV = (int) Core.mean(region3_HSV).val[0];
            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    leftSpikeTopLeft, // First point which defines the rectangle
                    leftSpikeBottomRight, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    lineThickness); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    middleSpikeTopLeft, // First point which defines the rectangle
                    middleSpikeBottomRight, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    lineThickness); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    rightSpikeTopLeft, // First point which defines the rectangle
                    rightSpikeBottomRight, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    lineThickness); // Thickness of the rectangle lines


            /*
             * Find the min of the 3 averages
             */
            int maxOneTwo = Math.max(leftRegionAvgHSV, centerRegionAvgHSV);
            int max = Math.max(maxOneTwo, rightRegionAvgHSV);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if (max == leftRegionAvgHSV) // Was it from region 1?
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
                        lineThickness); // Negative thickness means solid fill
            }
            else if(max == centerRegionAvgHSV) // Was it from region 2?
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
                        lineThickness); // Negative thickness means solid fill
            } else if (max == rightRegionAvgHSV) // Was it from region 3?
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
                        lineThickness); // Negative thickness means solid fill
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