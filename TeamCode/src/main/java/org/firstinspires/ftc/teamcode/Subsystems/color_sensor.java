package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import java.util.HashMap;
import java.util.Map;

public class color_sensor extends Subsystem {

    public static final color_sensor INSTANCE = new color_sensor();
    public color_sensor() {

    }
    private ColorSensor color_sensor;
    private Map<Integer, Integer> color_mappings = new HashMap<>();


    // Yellow hue
    int common_color = 60;
    // Red hue
    int alliance_color = 0;

    public boolean check_color() {
        int detected_color = color_sensor.argb();
        if (((detected_color > common_color-10) && (detected_color > common_color+10)) || ((detected_color > alliance_color-10) && (detected_color < alliance_color+10))) {
            return true;
        } else {
            return false;
        }
    }
    public boolean check_specific_color(int target) {
        int sensed_color = color_sensor.argb();
        int target_color = color_mappings.get(target);

        if ((sensed_color > target_color-10) && (sensed_color < target_color+10)) {
            return true;
        } else {
            return false;
        }
    }
    @Override()
    public void initialize() {
        color_sensor = OpModeData.INSTANCE.getHardwareMap().get(ColorSensor.class, "sensor_color");
        //yellow, red, blue
        color_mappings.put(0, 60);
        color_mappings.put(1, 0);
        color_mappings.put(2, 240);
    }
}
