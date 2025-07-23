package org.firstinspires.ftc.teamcode.Subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

public class extendo extends Subsystem {
    public static final extendo INSTANCE = new extendo();

    public extendo() {

    }

    // Initialization
    public MotorEx extendo_motor;
    public PIDFController extendo_controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));

    public String extendo_config = "motor_extendo";

    public Command extension_power(float power) {
        return new SetPower(extendo_motor,
                power,
                this);
    }

    public Command transfer() {
        return new RunToPosition(extendo_motor,
                0.0,
                extendo_controller,
                this);
    }

    @Override()
    // Initialize function
    public void initialize() {
        extendo_motor = new MotorEx(extendo_config);
    }
}
