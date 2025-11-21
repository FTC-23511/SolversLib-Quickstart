package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ColorSensorsSubsystem extends SubsystemBase {
    private NormalizedColorSensor colorSensor1; // Color sensor in pos 1. see spindexer subsystem comment

    private NormalizedColorSensor colorSensor2; // Color sensor in pos 2. see spindexer subsystem comment

    public final static float[] greenHigherHSV = {174f, 0.86f, 0.31f};
    public final static float[] greenLowerHSV  = {134f, 0.46f, 0.0f};

    public final static float[] purpleHigherHSV = {238.11765f, 0.5617021f, 0.29681849f};
    public final static float[] purpleLowerHSV  = {198.11765f, 0.1617021f, 0.0f};

    public ColorSensorsSubsystem(final HardwareMap hMap) {
        colorSensor1 = hMap.get(NormalizedColorSensor.class, "colorsensor1");
        colorSensor1.setGain(27.0f);

        colorSensor2 = hMap.get(NormalizedColorSensor.class, "colorsensor2");
        colorSensor2.setGain(27.0f);
    }
    /**
    * @param sensorNum Sensor num- 1 is left, num- 2 is right if you give anything else it will default to the left
     * @return float[] HSV, hardware call to the color sensors
     */
    public float[] senseColorsHSV(int sensorNum) {
        NormalizedRGBA normalizedColors = null;

        //Select which NormalizedRGBA color sensor
        if (sensorNum == 1) {
            normalizedColors = colorSensor1.getNormalizedColors();
        } else if (sensorNum == 2) {
            normalizedColors = colorSensor2.getNormalizedColors();
        } else {
            normalizedColors = colorSensor1.getNormalizedColors();
        }
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
    private static boolean colorIsGreen(float[] colorsHSV) {
        return colorInRange(colorsHSV, greenLowerHSV, greenHigherHSV);
    }
    private static boolean colorIsPurple(float[] colorsHSV) {
        return colorInRange(colorsHSV, purpleLowerHSV, purpleHigherHSV);
    }
    private static boolean colorIsWhite(float[] colorsHSV) {
        return colorInRange(colorsHSV, new float[]{0f, 0.99f, 0.99f}, new float[]{360f, 1f, 1f});
    }

    /**
    * @param sensorNum Takes in an int that tells it which color it needs to check: 1 is left, 2 is right, 3 or anything else will default to both (if any of them sense the color)
     **/
    public boolean checkIfGreen(int sensorNum) {
        if (sensorNum > 2 || sensorNum < 1) {
            return colorIsGreen(senseColorsHSV(1)) || colorIsGreen(senseColorsHSV(2));
        }
        return colorIsGreen(senseColorsHSV(sensorNum));
    }
    /**
     * @param sensorNum Takes in an int that tells it which color it needs to check: 1 is left, 2 is right, 3 or anything else will default to both (if any of them sense the color)
     **/
    public boolean checkIfPurple(int sensorNum) {
        if (sensorNum > 2 || sensorNum < 1) {
            return colorIsPurple(senseColorsHSV(1)) || colorIsPurple(senseColorsHSV(2));
        }
        return colorIsPurple(senseColorsHSV(sensorNum));
    }
    /**
     * @param sensorNum Takes in an int that tells it which color it needs to check: 1 is left, 2 is right, 3 or anything else will default to both (if any of them sense the color)
     **/
    public boolean checkIfWhite(int sensorNum) {
        if (sensorNum > 2 || sensorNum < 1) {
            return colorIsWhite(senseColorsHSV(1)) || colorIsWhite(senseColorsHSV(2));
        }
        return colorIsWhite(senseColorsHSV(sensorNum));
    }
    public static RobotConstants.BallColors colorsHSVToBallsColors(float[] colorsHSV) {
        if (colorIsGreen(colorsHSV))  return RobotConstants.BallColors.GREEN;
        if (colorIsPurple(colorsHSV)) return RobotConstants.BallColors.PURPLE;
        if (colorIsWhite(colorsHSV))  return RobotConstants.BallColors.UNKNOWN;
        return RobotConstants.BallColors.NONE;
    }






    public static boolean colorInRange(float[] color, float[] min, float[] max) {
        return
                min[0] <= color[0] && color[0] <= max[0] && //Red is within min and max range
                        min[1] <= color[1] && color[1] <= max[1] && //Green is within min and max range
                        min[2] <= color[2] && color[2] <= max[2];   //brue is within the range,
    }
}