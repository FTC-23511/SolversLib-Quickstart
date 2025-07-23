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

public class wrist extends Subsystem {
    public static final wrist INSTANCE = new wrist();

    public wrist() {

    }

    //Init
    public Servo wrist_servo;

    public String wrist_config = "servo_wrist";


    public Command intake() {
        return new ServoToPosition(wrist_servo,
                0.33,
                this);
    }


    public Command spec() {
        return new ServoToPosition(wrist_servo,
                0.0,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        wrist_servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, wrist_config);
        wrist_servo.setPosition(0.33);
    }
}