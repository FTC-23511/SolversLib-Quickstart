package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import com.seattlesolvers.solverslib.command.CommandBase;

import Subsistemas.ShooterSubsystem;

public class SpinUpShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final double targetVelocity;
    private final double tolerance; // ticks per second

    public SpinUpShooterCommand(ShooterSubsystem shooter, double targetVelocity, double tolerance) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;
        this.tolerance = tolerance;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetVelocity(targetVelocity);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return shooter.isAtTarget(tolerance);
    }
}