package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import com.seattlesolvers.solverslib.command.CommandBase;

import Subsistemas.IntakeSubsystem_Autonomous;

public class SetIntakeVelocityCommand extends CommandBase {
    private final IntakeSubsystem_Autonomous intake;
    private final double Velocity;

    /**
     * Comando para establecer la potencia del intake
     * @param intake Subsistema del intake
     * @param Velocity Velocidad del motor (ej: 1400, 1200, 200)
     */
    public SetIntakeVelocityCommand(IntakeSubsystem_Autonomous intake, double Velocity) {
        this.intake = intake;
        this.Velocity = Velocity;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPower(Velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}