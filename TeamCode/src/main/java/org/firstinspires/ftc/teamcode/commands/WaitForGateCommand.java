package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

public class WaitForGateCommand extends CommandBase {
    private final GateSubsystem gateSubsystem;
    public WaitForGateCommand(GateSubsystem gateSubsystem) {
        this.gateSubsystem = gateSubsystem;
    }

    @Override
    public boolean isFinished() {
        return gateSubsystem.isAtTarget();
    }
}
