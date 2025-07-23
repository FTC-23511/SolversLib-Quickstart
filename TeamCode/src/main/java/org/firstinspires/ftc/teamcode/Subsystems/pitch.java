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

public class pitch extends Subsystem {
    public static final pitch INSTANCE = new pitch();

    public pitch() {

    }

    //Init
    public Servo pitch_servo;

    public String pitch_config = "servo_pitch";


    public Command intake() {
        return new ServoToPosition(pitch_servo,
                0.39,
                this);
    }


    public Command reject() {
        return new ServoToPosition(pitch_servo,
                0.27,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        pitch_servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, pitch_config);
        pitch_servo.setPosition(0.0);
    }
}