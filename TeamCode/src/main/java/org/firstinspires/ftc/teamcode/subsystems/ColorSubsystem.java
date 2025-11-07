package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Arrays;

public class ColorSubsystem extends SubsystemBase {
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;
    private NormalizedRGBA sensedcolors;
    public float[] colorHSV = {0, 0, 0}; //defaut
    public float[] greenHigherHSV = {175f, 0.9f, 0.95f};  // upper bound for bright/lime greens
    public float[] greenLowerHSV  = {135f, 0.2f, 0.2f};   // lower bound for darker/olive greens

    public float[] purpleHigherHSV = {248f, 0.9f, 0.95f}; // upper bound for bright magenta/purple
    public float[] purpleLowerHSV  = {208f, 0.2f, 0.28f}; // lower bound for darker violet shades

    public ColorSubsystem(final HardwareMap hMap) {
        colorSensor1 = hMap.get(NormalizedColorSensor.class, "colorsensor1"); // update colorsensor names on console
        colorSensor2 = hMap.get(NormalizedColorSensor.class, "colorsensor2");
        colorSensor3 = hMap.get(NormalizedColorSensor.class, "colorsensor3");

        //if there is a controllable light turn it on
        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(true);
        }
        colorSensor1.setGain(105.5f);

        if (colorSensor2 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor2).enableLight(true);
        }
        colorSensor2.setGain(105.5f);

        if (colorSensor3 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor3).enableLight(true);
        }
        colorSensor3.setGain(105.5f);
    }


    public float[] senseColor(int positionToCheck) {
        if (positionToCheck == 1) {
            sensedcolors = colorSensor1.getNormalizedColors();
        } else if (positionToCheck == 2) {
            sensedcolors = colorSensor2.getNormalizedColors();
        } else {
            sensedcolors = colorSensor3.getNormalizedColors();
        }

        float r = sensedcolors.red;
        float g = sensedcolors.green;
        float b = sensedcolors.blue;

        // Convert RGB to HSV
        float[] rgb = {r, g, b};
        colorHSV = rgbToHsv(rgb);

        //this is debug I think but you can only see this in logcat so I guess it isn't useful
        //Log.d("colorsensor","Converted HSV: Hue=" + colorHSV[0] + ", Saturation=" + colorHSV[1] + ", Value=" + colorHSV[2]);
        return new float[] {colorHSV[0],colorHSV[1],colorHSV[2]};
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
        Log.d("colorsensor", "RGB to HSV: R=" + r + ", G=" + g + ", B=" + b + " => H=" + h + ", S=" + s + ", V=" + v);

        return new float[] {h, s, v};
    }

    public boolean checkIfGreen(int posToCheck) {
        senseColor(posToCheck);
        return colorInRange(colorHSV, greenLowerHSV, greenHigherHSV);
    }

    public boolean checkIfPurple(int posToCheck) {
        senseColor(posToCheck);
        return colorInRange(colorHSV, purpleLowerHSV, purpleHigherHSV);
    }
    public boolean checkIfWhite(int posToCheck) {
        senseColor(posToCheck);
        return Arrays.equals(colorHSV, new float[]{0.0f, 0.0f, 1.0f});
    }

    enum Colors {
        GREEN, PURPLE, WHITE, NONE
    }
    //unfinished
    public Colors[] getColorSensors() {
        ArrayList<Colors[]> colorArr = new ArrayList<>();
        for (int i = 1; i <= 3; i++) {
            if (checkIfGreen(i)) {
                colorArr.set(i, Colors.GREEN);
            } else if (checkIfPurple(i)) {
                colorArr[i] = Colors.PURPLE;
            } else if (checkIfWhite(i)) {
                colorArr[i] = Colors.WHITE;
            } else {
                colorArr[i] = Colors.NONE;
            }
        }

        return colorArr;
    }

    public boolean colorInRange(float[] color, float[] min, float[] max) {
        return
                min[0] <= color[0] && color[0] <= max[0] && //Red is within min and max range
                        min[1] <= color[1] && color[1] <= max[1] && //Green is within min and max range
                        min[2] <= color[2] && color[2] <= max[2];   //brue is within the range,
    }


}