package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class LoadBallCommand extends CommandBase {
    private SpindexerSubsystem spindexerSubsystem;
    private ColorSensorsSubsystem colorSensorsSubsystem; //idk if we need, delete if not
    private RobotConstants.ballColors targetColor;
    public LoadBallCommand(SpindexerSubsystem spindexerSubsystem, ColorSensorsSubsystem colorSensorsSubsystem, RobotConstants.ballColors targetColor) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.colorSensorsSubsystem = colorSensorsSubsystem;
        this.targetColor = targetColor;
        addRequirements(colorSensorsSubsystem, spindexerSubsystem);
    }

    @Override
    public void initialize() {
        //TODO (medium): implement this command
        //Cycle balls until the target color is in position 3
        //edge case: if none of them are the target color, cycle until an UNKNOWN color is in position 3
        //if none of them are the target color or UNKNOWN, dont do anything
        //if targetColor is UNKNOWN, then that means supply any ball at all -> dont do anything

    }
    @Override
    public boolean isFinished() {
        //TODO: return true when spindexer is done (?) maybe???
    }
}
