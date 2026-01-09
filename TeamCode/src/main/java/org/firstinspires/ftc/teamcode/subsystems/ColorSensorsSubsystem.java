package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class ColorSensorsSubsystem extends SubsystemBase {
    private NormalizedColorSensor intakeSensor1;
    private NormalizedRGBA intakeSensor1Result = null;
    private NormalizedColorSensor intakeSensor2;
    private NormalizedRGBA intakeSensor2Result = null;

    private NormalizedColorSensor backSensor;
    private NormalizedRGBA backResult = null;


    public final static float[] intakeGreenHigherHSV = {174f, 0.86f, 0.31f};
    public final static float[] intakeGreenLowerHSV  = {134f, 0.46f, 0.0f};

    public final static float[] intakePurpleHigherHSV = {238.11765f, 0.5617021f, 0.29681849f};
    public final static float[] intakePurpleLowerHSV  = {198.11765f, 0.1617021f, 0.0f};

    public final static float[] backGreenHigherHSV = {174f, 0.86f, 0.31f};
    public final static float[] backGreenLowerHSV  = {134f, 0.46f, 0.0f};

    public final static float[] backPurpleHigherHSV = {238.11765f, 0.5617021f, 0.29681849f};
    public final static float[] backPurpleLowerHSV  = {198.11765f, 0.1617021f, 0.0f};

    public final static float[] whiteLowerHSV = {0f, 0.99f, 0.99f};
    public final static float[] whiteHigherHSV = {360f, 1f, 1f};


    public ColorSensorsSubsystem(final HardwareMap hMap) {
        intakeSensor1 = hMap.get(NormalizedColorSensor.class, "colori1");
        intakeSensor1.setGain(27.0f);

        intakeSensor2 = hMap.get(NormalizedColorSensor.class, "colori2");
        intakeSensor2.setGain(27.0f);

        backSensor = hMap.get(NormalizedColorSensor.class, "colorb");
        backSensor.setGain(27.0f);
    }
    public NormalizedRGBA getIntakeSensor1Result() {
        return intakeSensor1Result;
    }
    public NormalizedRGBA getIntakeSensor2Result() {
        return intakeSensor2Result;
    }
    public NormalizedRGBA getBackResult() {
        return backResult;
    }
    public void updateSensor1() {
        intakeSensor1Result = intakeSensor1.getNormalizedColors();
    }
    public void updateSensor2() {
        intakeSensor2Result = intakeSensor2.getNormalizedColors();
    }
    public void updateBack() {
        backResult = backSensor.getNormalizedColors();
    }

    // Function to convert RGB to HSV
    public static float[] rgbToHsv(float[] colors) {
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


    private static boolean colorInRange(float[] color, float[] min, float[] max) {
        return
                min[0] <= color[0] && color[0] <= max[0] && //Red is within min and max range
                        min[1] <= color[1] && color[1] <= max[1] && //Green is within min and max range
                        min[2] <= color[2] && color[2] <= max[2];   //brue is within the range,
    }
}