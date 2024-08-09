package org.firstinspires.ftc.teamcode.subsystem.vision;


import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

/**
 * This class provides the APIs to detect the location of the team elements
 */
public  class HSVTeamElementDetector {
    public static int leftRegionAvgHSV;
    public static int centerRegionAvgHSV;
    public static int rightRegionAvgHSV;
    TeamPropDeterminationPipeline pipeline;
    static int offset;
    public static int[] counts;

    HardwareMap hardwareMap; ///< hardware map
    Telemetry telemetry; ///< telemetry logger


    /**
     * the full constructor
     *
     * @param _hardwareMap the hardware map
     * @param _telemetry the telemetry logger
     * @param _startPos the starting
     */
    OpenCvWebcam webCam;


    public HSVTeamElementDetector(HardwareMap _hardwareMap, Telemetry _telemetry, String _startPos) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
        counts = new int[]{0, 0, 0};
        //_startPos = new Vector2d(17,-9);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        char detectColor = 'b';

        if ((_startPos.equals("redBack")) || (_startPos.equals("redFront"))){
            detectColor = 'r';
        }

        if ((_startPos.equals("blueBack")) || (_startPos.equals("blueFront"))){
            offset = 80; // For Blue Teamprop
        } else {
            offset = 130; // For Red Teamprop
        }

        pipeline = new TeamPropDeterminationPipeline(detectColor);
        webCam.setPipeline(pipeline);
        webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
//                ExposureControl;
                //webCam.getExposureControl();
                webCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera could not be opened.");
                telemetry.addData("Error Code:", errorCode);
                //This will be called if the camera could not be opened

            }
        });
    }

    public void startDetection(boolean runDetection) {
//        pipeline.runDetection = runDetection;
        counts[0]=0;
        counts[1]=0;
        counts[2]=0;
    }

    /**
     * Get the position of the team element
     *
     * @return the field coordinate of the team element (if found) | null (otherwise)
     */

    public String getTeamElementPosition() {
        // Find which stripe the team element is placed on
        // convert the stripe index to the stripe coordinate based on the start position
        String objectArea = "CENTER";
        // TODO: to be implemented later
//        for(int i= 0;  i<11; i++){
        objectArea = String.valueOf(pipeline.getAnalysis());
//        }

        telemetry.addData("Left Region cr", leftRegionAvgHSV);
        telemetry.addData("Center Region cr", centerRegionAvgHSV);
        telemetry.addData("Right Region cr", rightRegionAvgHSV);
        String countArray = "[" + counts[0]+", " + counts[1]+", " + counts[2]+"]";
        telemetry.addData("[Left Readings; Center Readings; Right Readings] \n", countArray);
        telemetry.update();

        if ((leftRegionAvgHSV > centerRegionAvgHSV) && (leftRegionAvgHSV > rightRegionAvgHSV)) {
            objectArea = "LEFT";
        }
        else if ((centerRegionAvgHSV > leftRegionAvgHSV) && (centerRegionAvgHSV > rightRegionAvgHSV)) {
            objectArea = "CENTER";
        }
        else if ((rightRegionAvgHSV > leftRegionAvgHSV) && (rightRegionAvgHSV > centerRegionAvgHSV)) {
            objectArea = "RIGHT";
        }

        else {
            objectArea = "LEFT";
        }
        return objectArea;
    }

//    @Override
//    public Mat processFrame(Mat input) {
//        return null;
//    }

    public static class TeamPropDeterminationPipeline extends OpenCvPipeline {
        static int detectChannel;
        static boolean runDetection = false;

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

        public TeamPropDeterminationPipeline(char color) {
            if (color == 'r' || color == 'R') {
                detectChannel = 1;
            } else { // (color == 'b' || color == 'B')
                detectChannel = 2;
            }
        }

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile TeamPropPosition position = TeamPropPosition.RIGHT;
        int lineThickness = 4;
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cr channel to the 'Cr' variable
         */
        void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);// convert to HSV
            if (detectChannel == 1){
                Core.inRange(HSV, RedHighLowerBound, RedHighUpperBound, RedHigh);// Top Red Values
                Core.inRange(HSV, RedLowLowerBound, RedLowUpperBound, RedLow);// Lower Red Values
                Core.add(RedLow, RedHigh, targetColorPixels); //combines both sections of red pixels into one Mat
            }else{
                Core.inRange(HSV, BlueLowerBound, BlueUpperBound, targetColorPixels);// Blue Values
            }
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
                counts[0]++;

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
            } else if (max == centerRegionAvgHSV) // Was it from region 2?
            {
                position = TeamPropPosition.CENTER; // Record our analysis
                counts[1]++;

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
                counts[2]++;

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

            int maxInd = -1;
            int maxCount = -1;

            for (int i = 0; i < counts.length; i++) {
                if (counts[i] > maxCount) {
                    maxInd = i;
                    maxCount = counts[i];
                }
            }

            final TeamPropPosition[] positions = {
                    TeamPropPosition.LEFT,
                    TeamPropPosition.CENTER,
                    TeamPropPosition.RIGHT
            };
            return positions[maxInd];
        }

        public void getAverage(int avg, Mat regionCr) {

            avg = (int) Core.mean(regionCr).val[0];
        }
    }
}