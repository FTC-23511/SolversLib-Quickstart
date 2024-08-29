package org.firstinspires.ftc.teamcode.tuning.example;


import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;

import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonLynxDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;

public class ExampleHardware {
    public Servo leftServo;
    public Servo rightServo;
    public Servo centerServo;
    public SolversMotor intakeMotor;

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
//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        rightServo = hardwareMap.get(Servo.class, "rightServo");
//
//        leftServo.setDirection(Servo.Direction.REVERSE);
//        centerServo = hardwareMap.get(Servo.class, "centerServo");
//        intakeMotor = (PhotonDcMotor) hardwareMap.dcMotor.get("intake");
//        intakeMotor = new SolversMotor(hardwareMap.get(PhotonDcMotor.class, "intake"), 0.01);
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
}
