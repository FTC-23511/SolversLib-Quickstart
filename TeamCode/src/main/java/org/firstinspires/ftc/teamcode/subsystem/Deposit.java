package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;

public class Deposit {
    public Servo leftArm;
    public Servo rightArm;

    public Servo leftClaw;
    public Servo rightClaw;

    public Servo wrist;

    private final double[] wristPositions = {1, 0.82, 0.64, 0.46, 0.28, 0.08};
    private int wristSplice = 0;



    public Deposit(HardwareMap hardwareMap) {

        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        wrist = hardwareMap.get(Servo.class, "wrist");

        rightArm.setDirection(Servo.Direction.REVERSE);
    }

    public double getWristPos() {
        return (wristPositions[wristSplice]);
    }

    public void moveWristLeft() {
        if (wristSplice != 0) {
            wristSplice -= 1;
        }
        wrist.setPosition(getWristPos());
    }

    public void moveWristRight() {
        if (wristSplice != (wristPositions.length - 1)) {
            wristSplice += 1;
        }
        wrist.setPosition(getWristPos());
    }
}