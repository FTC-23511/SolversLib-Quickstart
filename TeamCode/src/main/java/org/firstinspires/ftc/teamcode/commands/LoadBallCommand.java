package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.RobotConstants.SPINDEXER_TICKS_PER_DEG;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class LoadBallCommand extends CommandBase { //Assumes gate is up - pls
    private SpindexerSubsystem spindexerSubsystem;
    private RobotConstants.BallColors targetColor;
    public LoadBallCommand(SpindexerSubsystem spindexerSubsystem, RobotConstants.BallColors targetColor) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.targetColor = targetColor;
        addRequirements(spindexerSubsystem);
    }

    @Override
    public void initialize() {
        RobotConstants.BallColors[] balls = spindexerSubsystem.getBalls();

        boolean ballsContainsTarget = false;
        for (RobotConstants.BallColors color : balls) { //idk why java doesnt have a .contains method for arrays :(
            if (color == targetColor) {
                ballsContainsTarget = true;
                break;
            }
        }
        if (!ballsContainsTarget) {
            targetColor = RobotConstants.BallColors.UNKNOWN;
        }

        if (balls[1] == targetColor) {
            spindexerSubsystem.moveSpindexerBy(SPINDEXER_TICKS_PER_DEG * 120);
            spindexerSubsystem.shiftBallsArrayBy(1);
        } else if (balls[0] == targetColor) {
            spindexerSubsystem.moveSpindexerBy(SPINDEXER_TICKS_PER_DEG * 240); //Forward because it will fly out of intake
            spindexerSubsystem.shiftBallsArrayBy(2);
        }
    }
    @Override
    public boolean isFinished() {
        return (spindexerSubsystem.isNearTargetPosition() && spindexerSubsystem.isLowVelocity());
    }
}
