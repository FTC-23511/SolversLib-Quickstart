package org.firstinspires.ftc.teamcode.tuning.servoTemplates;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Subsystem {
    public Servo centerServo;
    public Servo leftServo;
    public Servo rightServo;
    private DcMotorEx arm;

    public Subsystem(HardwareMap hardwareMap) {
        centerServo = hardwareMap.get(Servo.class, "centerServo");

//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        rightServo = hardwareMap.get(Servo.class, "rightServo");

//        rightServo.setDirection(Servo.Direction.REVERSE);

//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setDirection(DcMotorSimple.Direction.FORWARD);

    }
}
