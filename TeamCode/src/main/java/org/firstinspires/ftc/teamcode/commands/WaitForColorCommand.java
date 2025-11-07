package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;

public class WaitForColorCommand extends CommandBase {
    private final ColorSubsystem colorSubsystem;
    public enum ColorToCheck {
        EITHER, GREEN, PURPLE
    };
    private ColorToCheck colortocheck = ColorToCheck.EITHER;
    public WaitForColorCommand(ColorSubsystem colorSubsystem) {
        this.colorSubsystem = colorSubsystem;
        colortocheck = ColorToCheck.EITHER;
        addRequirements(colorSubsystem);
    }
    public WaitForColorCommand(ColorSubsystem colorSubsystem, ColorToCheck colortocheck) {
        this.colorSubsystem = colorSubsystem;
        this.colortocheck = colortocheck;
        addRequirements(colorSubsystem);
    }
    @Override
    public boolean isFinished() {
        return colorSubsystem.checkIfGreen(1) || colorSubsystem.checkIfPurple(1) || colorSubsystem.checkIfWhite(1);
    }
}
