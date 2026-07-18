package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import Subsistemas.ServosSubsystem_Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;

public class CloseTopeCommand extends CommandBase {

    private final ServosSubsystem_Autonomous servos;

    public CloseTopeCommand(ServosSubsystem_Autonomous servos) {
        this.servos = servos;
        addRequirements(servos);
    }

    @Override
    public void initialize() {
        servos.TopeCerrado();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}