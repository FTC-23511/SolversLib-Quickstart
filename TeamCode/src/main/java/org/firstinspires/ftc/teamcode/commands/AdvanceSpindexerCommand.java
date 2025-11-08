package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.ArrayList;

public class AdvanceSpindexerCommand extends CommandBase {
    private ColorSubsystem colorSubsystem;
    private SpindexerSubsystem spindexer;
    private ArrayList<ColorSubsystem.Colors> colors;
    private int places;

    public AdvanceSpindexerCommand(ColorSubsystem colorSubsystem, SpindexerSubsystem spindexer, ArrayList<ColorSubsystem.Colors> colors, int places) {
        this.colorSubsystem = colorSubsystem;
        this.spindexer = spindexer;
        this.colors = colors;
        this.places = places;

        ArrayList<ColorSubsystem.Colors> colorArr = colors;
        addRequirements(colorSubsystem);
        addRequirements(spindexer);
    }

    @Override
    public void execute() {
        spindexer.advanceSpindexer();
        colorSubsystem.shiftColors(colors, places);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
