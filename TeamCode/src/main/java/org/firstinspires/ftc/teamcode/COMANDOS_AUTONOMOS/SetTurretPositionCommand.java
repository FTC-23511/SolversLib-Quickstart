package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import com.seattlesolvers.solverslib.command.CommandBase;

import Subsistemas.TurretSubsystem_Autonomous;

public class SetTurretPositionCommand extends CommandBase {
    private final TurretSubsystem_Autonomous turret;
    private final int targetPosition;
    private final boolean waitForTarget;

    /**
     * Comando para mover la torreta a una posición
     * @param turret Subsistema de la torreta
     * @param targetPosition Posición objetivo en ticks (ej: 500, -1000, 0)
     * @param waitForTarget Si debe esperar a llegar a la posición
     */
    public SetTurretPositionCommand(TurretSubsystem_Autonomous turret,
                                    int targetPosition,
                                    boolean waitForTarget) {
        this.turret = turret;
        this.targetPosition = targetPosition;
        this.waitForTarget = waitForTarget;
        addRequirements(turret);
    }

    // Constructor simplificado que espera por defecto
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