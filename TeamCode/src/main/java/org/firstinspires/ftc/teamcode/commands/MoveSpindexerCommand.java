package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.RobotConstants.SPINDEXER_TICKS_PER_DEG;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class MoveSpindexerCommand extends CommandBase {
    private SpindexerSubsystem spindexerSubsystem;
    private GateSubsystem gateSubsystem;
    public int number;
    private boolean instant = false;
    public MoveSpindexerCommand(SpindexerSubsystem spindexerSubsystem, GateSubsystem gateSubsystem, int num, boolean instant) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.gateSubsystem = gateSubsystem;
        number = num;
        this.instant = instant;

    }

    @Override
    public void initialize() {
        if (Math.abs(gateSubsystem.DOWN - gateSubsystem.getPosition()) < 0.1) {
            spindexerSubsystem.setBallAt(2, RobotConstants.BallColors.NONE);
        }
        spindexerSubsystem.moveSpindexerBy(120 * number);
        spindexerSubsystem.shiftBallsArrayBy(number);
    }

    @Override
    public boolean isFinished() {
        if (instant) {
            return true;
        }
        return spindexerSubsystem.isNearTargetPosition();
    }
}
