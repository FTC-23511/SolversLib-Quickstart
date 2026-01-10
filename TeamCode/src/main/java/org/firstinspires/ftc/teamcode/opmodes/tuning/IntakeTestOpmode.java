package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Locale;

@Config
@TeleOp(name = "Intake Test", group = "Tuning")
public class IntakeTestOpmode extends OpMode {

    private IntakeSubsystem intake;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.set(IntakeSubsystem.IntakeState.INTAKING);
        } else if (gamepad1.b) {
            intake.set(IntakeSubsystem.IntakeState.REVERSE);
        } else {
            intake.set(IntakeSubsystem.IntakeState.STILL);
        }
    }

}