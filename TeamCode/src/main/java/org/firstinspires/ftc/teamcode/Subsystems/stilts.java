package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import java.util.ArrayList;
import java.util.List;

public class stilts extends Subsystem {
    public static final stilts INSTANCE = new stilts();

    public stilts() {

    }

    //Init
    public Servo stilt_l, stilt_r;
    public List<Servo> stilt_servos = new ArrayList<>();

    public String stilt_l_config = "servo_stilt_l";
    public String stilt_r_config = "servo_stilt_r";


    public Command stow() {
        return new MultipleServosToPosition(stilt_servos,
                0.55,
                this);
    }


    public Command raise() {
        return new MultipleServosToPosition(stilt_servos,
                0.0,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        stilt_l = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, stilt_l_config);
        stilt_r = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, stilt_r_config);
        stilt_l.setPosition(0.6);
        stilt_r.setDirection(Servo.Direction.REVERSE);
        stilt_l.setPosition(0.6);
        stilt_servos.add(stilt_r);
        stilt_servos.add(stilt_r);
    }
}