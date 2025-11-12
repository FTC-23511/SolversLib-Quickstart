package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ColorSensorsSubsystem extends SubsystemBase {
    private NormalizedColorSensor colorSensor1; // Color sensor in pos 1. see spindexer subsystem comment
    private NormalizedColorSensor colorSensor2; // pos 2

    public float[] greenHigherHSV = {175f, 0.9f, 0.95f};
    public float[] greenLowerHSV  = {135f, 0.2f, 0.2f};

    public float[] purpleHigherHSV = {248f, 0.9f, 0.95f};
    public float[] purpleLowerHSV  = {208f, 0.2f, 0.28f};

    public ColorSensorsSubsystem(final HardwareMap hMap) {
        colorSensor1 = hMap.get(NormalizedColorSensor.class, "colorsensor1");
        colorSensor2 = hMap.get(NormalizedColorSensor.class, "colorsensor1");
        colorSensor1.setGain(27.0f);
        colorSensor2.setGain(27.0f);
    }

    public float[] senseColorsHSV() {
        //get colors as NormaledRGBA object
        NormalizedRGBA sensor1NormalizedColors = colorSensor1.getNormalizedColors();
        NormalizedRGBA sensor2NormalizedColors = colorSensor2.getNormalizedColors();
        //convert to rgb float[]
        float[] sensor1rgb = {sensor1NormalizedColors.red, sensor1NormalizedColors.green, sensor1NormalizedColors.blue};
        float[] sensor2rgb = {sensor2NormalizedColors.red, sensor2NormalizedColors.green, sensor2NormalizedColors.blue};
        //convert to hsv float[]
        float[] sensor1hsv = rgbToHsv(sensor1rgb);
        float[] sensor2hsv = rgbToHsv(sensor2rgb);
        //build array: 0,1,2 sensor1hsv 3,4,5 sensor2hsv
        return new float[] {sensor1hsv[0],sensor1hsv[1],sensor1hsv[2],sensor2hsv[0],sensor2hsv[1],sensor2hsv[2]};
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





    public static boolean colorInRange(float[] color, float[] min, float[] max) {
        return
                min[0] <= color[0] && color[0] <= max[0] && //Red is within min and max range
                        min[1] <= color[1] && color[1] <= max[1] && //Green is within min and max range
                        min[2] <= color[2] && color[2] <= max[2];   //brue is within the range,
    }
}