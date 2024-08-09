/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.subsystem.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

/*
 * This OpMode helps calibrate a webcam or RC phone camera, useful for AprilTag pose estimation
 * with the FTC VisionPortal.   It captures a camera frame (image) and stores it on the Robot Controller
 * (Control Hub or RC phone), with each press of the gamepad button X (or Square).
 * Full calibration instructions are here:
 *
 *  https://ftc-docs.firstinspires.org/camera-calibration
 *
 * In Android Studio, copy this class into your "teamcode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * In OnBot Java, use "Add File" to add this OpMode from the list of Samples.
 */
@Disabled
@Autonomous(name = "Utility: Camera Frame Capture", group = "Utility")
public class UtilityCameraFrameCapture extends LinearOpMode
{
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
    final boolean USING_WEBCAM = false;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int resolutionWidth = 640;
    final int resolutionHeight = 480;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;

    @Override
    public void runOpMode()
    {
        VisionPortal visionPortal;

        if (USING_WEBCAM)
        {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(resolutionWidth, resolutionHeight))
                    .build();
        }
        else
        {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(INTERNAL_CAM_DIR)
                    .setCameraResolution(new Size(resolutionWidth, resolutionHeight))
                    .build();
        }

        while (!isStopRequested())
        {
            boolean x = true;//gamepad1.x;

            if (x && !lastX)
            {
                visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
                capReqTime = System.currentTimeMillis();
            }

            lastX = x;

            telemetry.addLine("######## Camera Capture Utility ########");
            telemetry.addLine(String.format(Locale.US, " > Resolution: %dx%d", resolutionWidth, resolutionHeight));
            telemetry.addLine(" > Press X (or Square) to capture a frame");
            telemetry.addData(" > Camera Status", visionPortal.getCameraState());

            if (capReqTime != 0)
            {
                telemetry.addLine("\nCaptured Frame!");
            }

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000)
            {
                capReqTime = 0;
            }

            telemetry.update();
        }
    }
    /*private void takePhoto (){
        Mat src = Imgcodecs.imread("VisionPortal-CameraFrameCapture-000000");
        Mat dst = Imgcodecs.imwrite("CameraFrameCapture0-HSV");
        //Mat dst1 = "CameraFrameCapture0-HSV-LowRed";
        //Mat dst2 = "CameraFrameCapture0-HSV-HighRed";
        //converts the file from an RGB file to HSV
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);

        //Gets red pixels (camera, not game element)
//        Scalar lowerBound = new Scalar(0,200, 30);
//        Scalar upperBound = new Scalar(5,255,255);
        //Core.inRange((src, lowerBound,upperBound, dst1));


    }
    /*@Override
    public Object processFrame (Mat frame, long captureTimeNanos){
        Point startPoint = (50, 50);
        Point endPoint = (200, 23);

        Color color = (12, 255, 12);
        int thickness = 2;
        Imgproc.rectangle(frame, startPoint, endPoint, color, thickness);
    }*/
    /*@Override
    public void onDrawFrame (Canvas canvas,int onScreenWidth, int onscreenHeight,
                             Bitmap.Config.ARGB_8888, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
        Bitmap myBitmap = Bitmap.createBitmap(Mat.width(), Mat.height);
    }*/
}