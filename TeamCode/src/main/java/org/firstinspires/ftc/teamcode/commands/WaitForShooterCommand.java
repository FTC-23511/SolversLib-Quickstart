package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class WaitForShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    public WaitForShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    @Override
    public boolean isFinished() {
        return (shooterSubsystem.getActualVelocity() - shooterSubsystem.getTargetVelocity() < -50) && (shooterSubsystem.getActualVelocity() - shooterSubsystem.getTargetVelocity() > 50);
    }
}
