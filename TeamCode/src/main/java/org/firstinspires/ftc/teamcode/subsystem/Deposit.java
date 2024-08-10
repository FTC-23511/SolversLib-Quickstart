package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class Deposit {
    public Servo leftArm;
    public Servo rightArm;

    public Servo rightClaw;
    public Servo leftClaw;

    public Servo wrist;

    private final double[] wristPositions = {1, 0.82, 0.64, 0.46, 0.28, 0.08};
    private int wristSplice = 0;



    public Deposit(HardwareMap hardwareMap) {

        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        wrist = hardwareMap.get(Servo.class, "wrist");

        rightArm.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        openLeftClaw();
        openRightClaw();
    }

    public double getWristPos() {
        return (wristPositions[wristSplice]);
    }

    public double getLeftClawPos() {
        return new BigDecimal((String.valueOf(leftClaw.getPosition()))).setScale(2, RoundingMode.HALF_UP).doubleValue();
    }

    public void moveWristLeft() {
        if (wristSplice != 0) {
            wristSplice -= 1;
        }
        wrist.setPosition(getWristPos());
    }
    // 0.07, 0.18
    public void moveWristRight() {
        if (wristSplice != (wristPositions.length - 1)) {
            wristSplice += 1;
        }
        wrist.setPosition(getWristPos());
    }

    public void openRightClaw() {
        // 1, 0.82, 0.08
        if (getWristPos() == 1 || getWristPos() == 0.82 || getWristPos() == 0.08) {
            rightClaw.setPosition(0.59);
        }
        // 0.64, 0.46, 0.28
        else {
            leftClaw.setPosition(0.07);
        }
    }

    public void closeRightClaw() {
        // 1, 0.82, 0.08
        if (getWristPos() == 1 || getWristPos() == 0.82 || getWristPos() == 0.08) {
            rightClaw.setPosition(0.74);
        }
        // 0.64, 0.46, 0.28
        else {
            leftClaw.setPosition(0.18);
        }
    }

    public void openLeftClaw() {
        // 1, 0.82, 0.08
        if (getWristPos() == 1 || getWristPos() == 0.82 || getWristPos() == 0.08) {
            leftClaw.setPosition(0.07);
        }
        // 0.64, 0.46, 0.28
        else {
            rightClaw.setPosition(0.59);
        }
    }

    public void closeLeftClaw() {
        // 1, 0.82, 0.08
        if (getWristPos() == 1 || getWristPos() == 0.82 || getWristPos() == 0.08) {
            leftClaw.setPosition(0.18);

        }
        // 0.64, 0.46, 0.28
        else {
            rightClaw.setPosition(0.74);
        }
    }

    public void toggleLeftClaw() {
    }
}
