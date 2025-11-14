package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;

public class WaitForColorCommand extends CommandBase {
    private final ColorSensorsSubsystem colorSubsystem;
    public enum ColorToCheck {
        EITHER, GREEN, PURPLE
    };
    private ColorToCheck colortocheck = ColorToCheck.EITHER;
    public WaitForColorCommand(ColorSensorsSubsystem colorSubsystem) {
        this.colorSubsystem = colorSubsystem;
        colortocheck = ColorToCheck.EITHER;
        addRequirements(colorSubsystem);
    }
    public WaitForColorCommand(ColorSensorsSubsystem colorSubsystem, ColorToCheck colortocheck) {
        this.colorSubsystem = colorSubsystem;
        this.colortocheck = colortocheck;
        addRequirements(colorSubsystem);
    }
    @Override
    public boolean isFinished() {
        //TODO: fix
        return colorSubsystem.checkIfGreen(1) || colorSubsystem.checkIfPurple(1) || colorSubsystem.checkIfWhite(1);
    }
}
