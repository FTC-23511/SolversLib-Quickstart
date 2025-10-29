package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ColorSubSystem;

public class WaitForGreenCommand extends CommandBase {
    private final ColorSubSystem colorSubsystem;
    public WaitForGreenCommand(ColorSubSystem colorSubsystem) {
        this.colorSubsystem = colorSubsystem;
        
    }
}
