package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.Feedforward;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadManager;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import java.util.ArrayList;
import java.util.List;

public class claw extends Subsystem {
    public static final claw INSTANCE = new claw();

    public claw() {

    }

    //Init
    public Servo claw_servo;

    public String claw_config = "servo_claw";


    public Command open() {
        return new ServoToPosition(claw_servo,
                0.32,
                this);
    }


    public Command close() {
        return new ServoToPosition(claw_servo,
                0.0,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        claw_servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, claw_config);
        claw_servo.setPosition(0.0);
        claw_servo.setDirection(Servo.Direction.REVERSE);
    }
} 
