package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import Subsistemas.ServosSubsystem_Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;

public class OpenTopeCommand extends CommandBase {

    private final ServosSubsystem_Autonomous servos;

    public OpenTopeCommand(ServosSubsystem_Autonomous servos) {
        this.servos = servos;
        addRequirements(servos);
    }

    @Override
    public void initialize() {
        servos.TopeAbierto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
