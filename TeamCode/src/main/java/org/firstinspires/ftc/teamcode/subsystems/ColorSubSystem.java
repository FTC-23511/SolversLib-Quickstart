package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class ColorSubSystem extends SubsystemBase {
    private NormalizedColorSensor colorSensor;
    private NormalizedRGBA sensedcolors;
    public float[] colorHSV = {0, 0, 0}; //defaut
    public float[] greenHigherHSV = {170, 1.0f, 0.7f};  // upper bound for bright/lime greens
    public float[] greenLowerHSV  = {90, 0.4f, 0.1f};   // lower bound for darker/olive greens

    public float[] purpleHigherHSV = {300, 1.0f, 0.7f}; // upper bound for bright magenta/purple
    public float[] purpleLowerHSV  = {250, 0.4f, 0.1f}; // lower bound for darker violet shades

    public ColorSubSystem(final HardwareMap hMap) {
        colorSensor = hMap.get(NormalizedColorSensor.class, "colorsensor");

        //if there is a controllable light turn it on
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }


    public float[] senseColor() {
        sensedcolors = colorSensor.getNormalizedColors();

        float r = sensedcolors.red;
        float g = sensedcolors.green;
        float b = sensedcolors.blue;
        // Convert RGB to HSV
        colorHSV = rgbToHsv(r, g, b);
        Log.d("colorsensor","Converted HSV: Hue=" + colorHSV[0] + ", Saturation=" + colorHSV[1] + ", Value=" + colorHSV[2]);
        return new float[] {colorHSV[0],colorHSV[1],colorHSV[2]};
    }

    // Function to convert RGB to HSV
    public float[] rgbToHsv(float r, float g, float b) {
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

    public boolean sensorIsGreen() {
        return colorInRange(colorHSV, greenLowerHSV, greenHigherHSV);
    }

    public boolean sensorIsPurple() {
        return colorInRange(colorHSV, purpleLowerHSV, purpleHigherHSV);
    }
    public boolean colorInRange(float[] color, float[] min, float[] max) {
        return
                min[0] <= color[0] && color[0] <= max[0] && //Red is within min and max range
                        min[1] <= color[1] && color[1] <= max[1] && //Green is within min and max range
                        min[2] <= color[2] && color[2] <= max[2];   //brue is within the range,
    }
}