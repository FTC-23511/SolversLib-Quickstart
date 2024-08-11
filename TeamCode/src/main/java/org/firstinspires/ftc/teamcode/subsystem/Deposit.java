package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.System.round;

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

    private final double leftClawOpenPos = 0.07;
    private final double leftClawClosePos = 0.18;

    private final double rightClawOpenPos = 0.59;
    private final double rightClawClosePos = 0.74;

    private final double[] wristPositions = {1, 0.82, 0.64, 0.46, 0.28, 0.08};
    private int wristSplice = 0;

    public Deposit(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        wrist = hardwareMap.get(Servo.class, "wrist");

        rightArm.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(leftClawOpenPos);
        rightClaw.setPosition(rightClawOpenPos);
    }

    public void moveWristLeft() {
        if (wristSplice != 0) {
            wristSplice -= 1;
        }
        wrist.setPosition(wristPositions[wristSplice]);
    }

    public void moveWristRight() {
        if (wristSplice != (wristPositions.length - 1)) {
            wristSplice += 1;
        }
        wrist.setPosition(wristPositions[wristSplice]);
    }

    public void toggleLeftClaw() {
        if (round(wrist.getPosition(), 2) == 0.64 || (round(wrist.getPosition(), 2) == 0.46 || (round(wrist.getPosition(), 2) == 0.28))) {
            if (round(rightClaw.getPosition(), 2) == rightClawClosePos) {
                rightClaw.setPosition(rightClawOpenPos);
            } else {
                rightClaw.setPosition(rightClawClosePos);
            }
        } else {
            if (round(leftClaw.getPosition(), 2) == leftClawClosePos) {
                leftClaw.setPosition(leftClawOpenPos);
            } else {
                leftClaw.setPosition(leftClawClosePos);
            }
        }
    }

    public void toggleRightClaw() {
        if (round(wrist.getPosition(), 2) == 0.64 || round(wrist.getPosition(), 2) == 0.46 || round(wrist.getPosition(), 2) == 0.28) {
            if (round(leftClaw.getPosition(), 2) == leftClawClosePos) {
                leftClaw.setPosition(leftClawOpenPos);
            } else {
                leftClaw.setPosition(leftClawClosePos);
            }
        } else {
            if (round(rightClaw.getPosition(), 2) == rightClawClosePos) {
                rightClaw.setPosition(rightClawOpenPos);
            } else {
                rightClaw.setPosition(rightClawClosePos);
            }
        }
    }
}
