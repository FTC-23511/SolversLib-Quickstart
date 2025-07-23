package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.claw;
import org.firstinspires.ftc.teamcode.Subsystems.extendo;
import org.firstinspires.ftc.teamcode.Subsystems.intake;
import org.firstinspires.ftc.teamcode.Subsystems.intake_arm;
import org.firstinspires.ftc.teamcode.Subsystems.lift;
import org.firstinspires.ftc.teamcode.Subsystems.lift_arm;
import org.firstinspires.ftc.teamcode.Subsystems.pitch;
import org.firstinspires.ftc.teamcode.Subsystems.pto;
import org.firstinspires.ftc.teamcode.Subsystems.stilts;
import org.firstinspires.ftc.teamcode.Subsystems.wrist;

@TeleOp(name = "Command OpMode")
public class command_tele_no_color extends PedroOpMode {
    public command_tele_no_color() {
        super(lift.INSTANCE, extendo.INSTANCE, claw.INSTANCE, pitch.INSTANCE, intake_arm.INSTANCE, lift_arm.INSTANCE, pto.INSTANCE, stilts.INSTANCE, wrist.INSTANCE, intake.INSTANCE);
    }

    // Hardware declaration
    public String flMotorConfig = "motor_f_l";
    public String frMotorConfig = "motor_f_r";
    public String blMotorConfig = "motor_b_l";
    public String brMotorConfig = "motor_b_r";

    public MotorEx fl_motor;
    public MotorEx fr_motor;
    public MotorEx bl_motor;
    public MotorEx br_motor;

    public MotorEx[] drive_motors;

    public MecanumDriverControlled manual_drive;

    @Override()
    public void onInit() {
        fl_motor = new MotorEx(flMotorConfig).reverse();
        fr_motor = new MotorEx(frMotorConfig);
        bl_motor = new MotorEx(blMotorConfig).reverse();
        br_motor = new MotorEx(brMotorConfig);

        drive_motors = new MotorEx[] {fl_motor, fr_motor, bl_motor, br_motor};
    }

    @Override()
    public void onStartButtonPressed() {
        manual_drive = new MecanumDriverControlled(drive_motors, gamepadManager.getGamepad1());
        manual_drive.invoke();
        gamepadManager.getGamepad2().getLeftStick().setDisplacedCommand((value) -> lift.INSTANCE.lift_power(value.component2()));
        gamepadManager.getGamepad2().getRightStick().setDisplacedCommand((value) -> extendo.INSTANCE.extension_power(value.component2()));

        gamepadManager.getGamepad2().getRightBumper().setPressedCommand(
                () -> new SequentialGroup(
                        new ParallelGroup(
                                intake_arm.INSTANCE.transfer(),
                                pitch.INSTANCE.reject(),
                                intake.INSTANCE.intake(0.35)
                        ),
                        new Delay(0.075),
                        new ParallelGroup(
                                claw.INSTANCE.open(),
                                wrist.INSTANCE.intake(),
                                lift_arm.INSTANCE.sample_intake()
                        ),
                        new Delay(0.15),
                        new ParallelGroup(
                                claw.INSTANCE.close(),
                                lift_arm.INSTANCE.sample_intake(),
                                wrist.INSTANCE.spec()
                        )
                )
        );
        gamepadManager.getGamepad2().getRightTrigger().setStateChangeCommand(
                value -> new ParallelGroup(
                        intake_arm.INSTANCE.intake(),
                        pitch.INSTANCE.intake(),
                        intake.INSTANCE.intake(value)
                )
        );
        gamepadManager.getGamepad2().getLeftTrigger().setStateChangeCommand(
                value -> new ParallelGroup(
                        intake_arm.INSTANCE.intake(),
                        pitch.INSTANCE.reject(),
                        intake.INSTANCE.outtake(value)
                )
        );
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(
                () -> new SequentialGroup(
                        stilts.INSTANCE.raise(),
                        lift.INSTANCE.lift_power(1),
                        new Delay(0.75),
                        pto.INSTANCE.engage()
                )
        );
        gamepadManager.getGamepad2().getA().setPressedCommand(
                () -> new SequentialGroup(
                        new ParallelGroup(
                                lift_arm.INSTANCE.spec_intake(),
                                claw.INSTANCE.open()
                        ),
                        new Delay(0.3),
                        claw.INSTANCE.close(),
                        lift_arm.INSTANCE.spec_dep()
                )
        );
        gamepadManager.getGamepad2().getY().setPressedCommand(
                () -> new SequentialGroup(
                        lift.INSTANCE.lift_power((float) 1),
                        // Time for lift to lift and reach the beam for hanging specs, needs to be tested
                        new Delay(0.2),
                        // Power necessary to counter gravity, needs to be tested
                        lift.INSTANCE.lift_power((float) 0.17),
                        claw.INSTANCE.open()
                )
        );
    }
}