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

public class lift_arm extends Subsystem {
    public static final lift_arm INSTANCE = new lift_arm();

    public lift_arm() {

    }

    //Init
    public Servo lift_l, lift_r;
    public List<Servo> lift_servos = new ArrayList<>();

    public String lift_l_config = "servo_lift_l";
    public String lift_r_config = "servo_lift_r";


    public Command spec_intake() {
        return new MultipleServosToPosition(lift_servos,
                0.145,
                this);
    }

    public Command sample_intake() {
        return new MultipleServosToPosition(lift_servos,
                0.92,
                this);
    }

    public Command sample_dep() {
        return new MultipleServosToPosition(lift_servos,
                0.77,
                this);
    }

    public Command spec_dep() {
        return new MultipleServosToPosition(lift_servos,
                0.792,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        lift_l = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, lift_l_config);
        lift_r = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, lift_r_config);
        lift_l.setPosition(0.145);
        lift_l.setDirection(Servo.Direction.REVERSE);
        lift_r.setPosition(0.145);
        lift_servos.add(lift_l);
        lift_servos.add(lift_r);
    }
}