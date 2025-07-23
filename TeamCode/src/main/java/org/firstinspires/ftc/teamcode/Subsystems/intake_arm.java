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

public class intake_arm extends Subsystem {
    public static final intake_arm INSTANCE = new intake_arm();

    public intake_arm() {

    }

    //Init
    public Servo intake_arm_servo;

    public String intake_arm_config = "servo_intake_arm";


    public Command intake() {
        return new ServoToPosition(intake_arm_servo,
                0.3,
                this);
    }


    public Command transfer() {
        return new ServoToPosition(intake_arm_servo,
                0.75,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        intake_arm_servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, intake_arm_config);
        intake_arm_servo.setPosition(0.0);
    }
}