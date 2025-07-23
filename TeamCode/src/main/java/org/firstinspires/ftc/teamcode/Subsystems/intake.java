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

public class intake extends Subsystem {
    public static final intake INSTANCE = new intake();

    public intake() {

    }

    //Init
    public Servo intake_servo;

    public String intake_config = "crservo_intake";

    //Right trigger
    public Command intake(double power) {
        // Limits range from [0.5, 1] for forward spin
        double intake_power = (power / 2) + 0.5;
        return new ServoToPosition(intake_servo,
                intake_power,
                this);
    }

    // Left Trigger
    public Command outtake(double power) {
        // Limits range from [0, 0.5] for backward spin
        double intake_power = (power / 2);
        return new ServoToPosition(intake_servo,
                intake_power,
                this);
    }



    @Override()
    //Init Function
    public void initialize() {
        intake_servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, intake_config);
        intake_servo.setPosition(0.5);
    }
}