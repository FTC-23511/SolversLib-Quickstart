package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ScanAndUpdateBallsCommand extends CommandBase {
    private ColorSensorsSubsystem colorSensorsSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    public ScanAndUpdateBallsCommand(SpindexerSubsystem spindexerSubsystem, ColorSensorsSubsystem colorSensorsSubsystem) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.colorSensorsSubsystem = colorSensorsSubsystem;
        addRequirements(colorSensorsSubsystem, spindexerSubsystem);
    }

    @Override
    public void initialize() {
            //TODO: sense color and update arrays w/ spiundexer's setBalls method
            //TODO (hard): how do we prevent false positives / negatives?
            //example on how to set balls below
            //spindexerSubsystem.setBalls(new RobotConstants.ballColors[]{RobotConstants.ballColors.GREEN, RobotConstants.ballColors.GREEN, RobotConstants.ballColors.GREEN});
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
