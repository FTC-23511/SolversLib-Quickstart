package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import Subsistemas.ServosSubsystem_Autonomous;
import com.seattlesolvers.solverslib.command.CommandBase;

public class SetHoodPositionCommand extends CommandBase {

    private final ServosSubsystem_Autonomous servos;
    private final double position;

    public SetHoodPositionCommand(ServosSubsystem_Autonomous servos, double position) {
        this.servos = servos;
        this.position = position;
        addRequirements(servos);
    }

    @Override
    public void initialize() {
        servos.PositionHood(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}