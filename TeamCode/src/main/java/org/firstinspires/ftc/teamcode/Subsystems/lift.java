package org.firstinspires.ftc.teamcode.Subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

public class lift extends Subsystem {

    public static final lift INSTANCE = new lift();

    private lift() {

    }

    // Initialization
    public MotorEx lift_motor_l;
    public MotorEx lift_motor_r;
    public MotorGroup lift_motors;
    public PIDFController lift_controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));

    public String lift_l_config = "motor_lift_l";
    public String lift_r_config = "motor_lift_r";

    public Command lift_power(float power) {
        return new SetPower(lift_motors,
                power,
                this);
    }

    public Command transfer() {
        return new RunToPosition(lift_motors,
                0.0,
                lift_controller,
                this);
    }

    public Command spec_dep() {
        return new RunToPosition(lift_motors,
                0.0,
                lift_controller,
                this);
    }

    @Override()
    // Initialize function
    public void initialize() {
        lift_motor_l = new MotorEx(lift_l_config).reverse();
        lift_motor_r = new MotorEx(lift_r_config);
        lift_motors = new MotorGroup(lift_motor_l, lift_motor_r);
    }
}
