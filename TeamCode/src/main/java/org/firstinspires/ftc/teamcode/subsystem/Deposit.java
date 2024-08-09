package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;

public class Deposit {
    public Servo leftWrist;
    public Servo rightWrist;

    public Servo wrist;

    public Deposit(HardwareMap hardwareMap) {

        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");

        wrist = hardwareMap.get(Servo.class, "wrist");

        rightWrist.setDirection(Servo.Direction.REVERSE);
    }
}