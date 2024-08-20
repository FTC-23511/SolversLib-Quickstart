package org.firstinspires.ftc.teamcode.tuning.example;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ExampleHardware {
    public Servo leftServo;
    public Servo rightServo;
    public Servo centerServo;

    private static ExampleHardware instance = null;
    public boolean enabled;

    public static ExampleHardware getInstance() {
        if (instance == null) {
            instance = new ExampleHardware();
        }
        instance.enabled = true;
        return instance;
    }

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        centerServo = hardwareMap.get(Servo.class, "centerServo");

    }
}
