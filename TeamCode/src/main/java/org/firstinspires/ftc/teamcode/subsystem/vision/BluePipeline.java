package org.firstinspires.ftc.teamcode.subsystem.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public BluePipeline(Telemetry t) {
        telemetry = t;
    } // constructor for the class to set up telemetry

    Mat mat = new Mat(); // declare a new matrix (computer representation of an image)
    Rect leftRect = new Rect(0, 230, 200, 117); // define our regions of interest (where the algorithm is focusing on) as rectangles
    Rect midRect = new Rect(420, 230, 530, 100);
    Rect rightRect = new Rect(1080, 230, 200, 117);
//    Rect leftRect = new Rect(370, 900, 350, 250); // define our regions of interest (where the algorithm is focusing on) as rectangles
//    Rect midRect = new Rect(390, 420, 350, 250);
//    Rect rightRect = new Rect(370, 50, 350, 250);

    final double PERCENT_THRESHOLD = 0.19; // define our threshold
    private int finalAnswer;
    Scalar blue = new Scalar(0, 0, 255); // define what the color of the rectangle outline is that appears on the output (blue)

    public enum Location {
        RIGHT,
        CENTER,
        LEFT
    }

    private Location location;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // change the color space from rgb to HSV (Hue, Saturation, Value)
        Scalar lowBlueBound = new Scalar(100, 10, 10); // set lower and upper bounds for the color we want to recognize (blue in this case)
        Scalar highBlueBound = new Scalar(170, 500, 500); //hue is out of 180 here, double it if searching it up online due to online websites going from 0-360

        Core.inRange(mat, lowBlueBound, highBlueBound, mat); // see which pixels are in our range, convert the pixels we're looking for into white, store it back to mat i think
        Mat left = mat.submat(leftRect); // create sub-matrices for our regions of interest
        Mat middle = mat.submat(midRect);
        Mat right = mat.submat(rightRect);

        double leftValue = Core.sumElems(left).val[0] / leftRect.area() / 255; // get the percentage of white pixels that are present
        double midValue = Core.sumElems(middle).val[0] / midRect.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / rightRect.area() / 255;

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]); // display the raw values to the driver hub
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);

        left.release();
        right.release();
        middle.release();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, leftRect, blue, 2); // draw the rectangles on the output matrix
        Imgproc.rectangle(mat, rightRect, blue, 2);
        Imgproc.rectangle(mat, midRect, blue, 2);

        if (leftValue < PERCENT_THRESHOLD && midValue < PERCENT_THRESHOLD && rightValue < PERCENT_THRESHOLD) {
            finalAnswer = 0;
        } else {
            if (leftValue > midValue && leftValue > rightValue) {
                finalAnswer = 1;
            } else if (midValue > leftValue && midValue > rightValue) {
                finalAnswer = 2;
            } else if (rightValue > leftValue && rightValue > midValue) {
                finalAnswer = 3;
            } else if (leftValue == midValue && leftValue == rightValue && midValue == rightValue) {
                finalAnswer = 4;
            } else {
                finalAnswer = 5;
            }
        }

        if (finalAnswer == 0) {
            telemetry.addData("Position", "None of the values are above the percent threshold (please do more testing) - PROCEEDING WITH CAUTION ");
            if (leftValue > midValue && leftValue > rightValue) {
                telemetry.addData("Position", "left lol");
                location = Location.LEFT;
                Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green
            } else if (midValue > leftValue && midValue > rightValue) {
                telemetry.addData("Position", "middle meh");
                location = Location.CENTER;
                Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);
            } else if (rightValue > leftValue && rightValue > midValue) {
                telemetry.addData("Position", "right rawr xD");
                location = Location.RIGHT;
                Imgproc.rectangle(mat, rightRect, new Scalar(0, 255, 0), 2);
            }

        } else if (finalAnswer == 1) {

            telemetry.addData("Position", "left lol");
            location = Location.LEFT;
            Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green

        } else if (finalAnswer == 3) {

            telemetry.addData("Position", "right rawr xD");
            location = Location.RIGHT;
            Imgproc.rectangle(mat, rightRect, new Scalar(0, 255, 0), 2);

        } else if (finalAnswer == 2) {

            telemetry.addData("Position", "middle meh");
            location = Location.CENTER;
            Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);

        } else if (finalAnswer == 4) {

            telemetry.addData("Debugging", "HOW TF DO THEY ALL EQUAL THE SAME");
        } else if (finalAnswer == 5) {

            telemetry.addData("Debugging", "Rethink and double check EVERYTHING");
        }
        telemetry.update();


        return mat;
    }

    public Location getLocation() {
        return location;
    }
}

