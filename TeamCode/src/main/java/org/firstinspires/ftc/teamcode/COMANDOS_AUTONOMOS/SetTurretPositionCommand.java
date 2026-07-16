package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import com.seattlesolvers.solverslib.command.CommandBase;

import Subsistemas.TurretSubsystem_Autonomous;

public class SetTurretPositionCommand extends CommandBase {
    private final TurretSubsystem_Autonomous turret;
    private final int targetPosition;
    private final boolean waitForTarget;

    public SetTurretPositionCommand(TurretSubsystem_Autonomous turret,
                                    int targetPosition,
                                    boolean waitForTarget) {
        this.turret = turret;
        this.targetPosition = targetPosition;
        this.waitForTarget = waitForTarget;
        addRequirements(turret);
    }
    public SetTurretPositionCommand(TurretSubsystem_Autonomous turret, int targetPosition) {
        this(turret, targetPosition, true);
    }

    @Override
    public void initialize() {
        turret.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        if (!waitForTarget) {
            return true;
        }
        return turret.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            turret.stop();
        }
    }
}