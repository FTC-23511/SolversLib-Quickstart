package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ColorSensorsSubsystem extends SubsystemBase {
    private NormalizedColorSensor colorSensor1; // Color sensor in pos 1. see spindexer subsystem comment

    public final static float[] greenHigherHSV = {175f, 0.9f, 0.95f};
    public final static float[] greenLowerHSV  = {135f, 0.2f, 0.2f};

    public final static float[] purpleHigherHSV = {248f, 0.9f, 0.95f};
    public final static float[] purpleLowerHSV  = {208f, 0.2f, 0.28f};

    public ColorSensorsSubsystem(final HardwareMap hMap) {
        colorSensor1 = hMap.get(NormalizedColorSensor.class, "colorsensor1");
        colorSensor1.setGain(27.0f);
    }
    /**
    * @param sensorNum Sensor num- 1 is intake - only one color sensor now
     */
    public float[] senseColorsHSV(int sensorNum) {
        //Select which NormalizedRGBA
        NormalizedRGBA normalizedColors = colorSensor1.getNormalizedColors();
        // build rgb array
        float[] rgb = {
                normalizedColors.red,
                normalizedColors.green,
                normalizedColors.blue
        };
        //return hsv values as float
        return rgbToHsv(rgb);
    }
    // Function to convert RGB to HSV
    public float[] rgbToHsv(float[] colors) {
        float r = colors[0];
        float g = colors[1];
        float b = colors[2];
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;
        float h = 0, s = 0, v = max; // set Value
        if (delta != 0) {
            //calc Saturation
            s = delta / max;
            //calc Hue
            if (r == max) {
                h = (g - b) / delta;
            } else if (g == max) {
                h = 2 + (b - r) / delta;
            } else {
                h = 4 + (r - g) / delta;
            }
            h *= 60; // comvert to 360

            if (h < 0) { // make sure it's always positive
                h += 360;
            }
        }
        return new float[] {h, s, v};
    }

    //Check if green, check if purp methods
    /**
    @param colorsHSV Takes in an array in the form of [hue 0-360, saturation 0-1, value 0-1]. No longer accepts a number for color sensor location
     */
    public static boolean checkIfGreen(float[] colorsHSV) {
        return colorInRange(colorsHSV, greenLowerHSV, greenHigherHSV);
    }
    public static boolean checkIfPurple(float[] colorsHSV) {
        return colorInRange(colorsHSV, purpleLowerHSV, purpleHigherHSV);
    }
    public static boolean checkIfWhite(float[] colorsHSV) {
        return colorInRange(colorsHSV, new float[]{0f, 0.99f, 0.99f}, new float[]{360f, 1f, 1f});
    }
    public static RobotConstants.BallColors colorsHSVToBallsColors(float[] colorsHSV) {
        if (checkIfGreen(colorsHSV))  return RobotConstants.BallColors.GREEN;
        if (checkIfPurple(colorsHSV)) return RobotConstants.BallColors.PURPLE;
        if (checkIfWhite(colorsHSV))  return RobotConstants.BallColors.UNKNOWN;
        return RobotConstants.BallColors.NONE;
    }






    public static boolean colorInRange(float[] color, float[] min, float[] max) {
        return
                min[0] <= color[0] && color[0] <= max[0] && //Red is within min and max range
                        min[1] <= color[1] && color[1] <= max[1] && //Green is within min and max range
                        min[2] <= color[2] && color[2] <= max[2];   //brue is within the range,
    }
}