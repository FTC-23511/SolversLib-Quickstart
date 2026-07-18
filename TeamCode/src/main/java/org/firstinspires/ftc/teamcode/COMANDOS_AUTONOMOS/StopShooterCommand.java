package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;
import com.seattlesolvers.solverslib.command.CommandBase;

import Subsistemas.ShooterSubsystem;

public class StopShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;

    public StopShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return true; // Se ejecuta una sola vez
    }
}