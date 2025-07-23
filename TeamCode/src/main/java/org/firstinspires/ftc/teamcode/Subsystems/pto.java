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

public class pto extends Subsystem {
    public static final pto INSTANCE = new pto();

    public pto() {

    }

    //Init
    public Servo pto_l, pto_r;
    public List<Servo> pto_servos = new ArrayList<>();

    public String pto_l_config = "servo_pto_l";
    public String pto_r_config = "servo_pto_r";


    public Command engage() {
        return new MultipleServosToPosition(pto_servos,
                0.17,
                this);
    }


    public Command disengage() {
        return new MultipleServosToPosition(pto_servos,
                0.28,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        pto_l = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, pto_l_config);
        pto_r = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, pto_r_config);
        pto_l.setPosition(0.28);
        pto_r.setDirection(Servo.Direction.REVERSE);
        pto_l.setPosition(0.28);
        pto_servos.add(pto_l);
        pto_servos.add(pto_r);
    }
}