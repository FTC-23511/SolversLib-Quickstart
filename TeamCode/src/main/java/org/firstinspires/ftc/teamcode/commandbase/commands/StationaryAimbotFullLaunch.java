package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class
StationaryAimbotFullLaunch extends ParallelCommandGroup {
    public StationaryAimbotFullLaunch(Robot robot) {
        addCommands(
                new InstantCommand(
                        () -> robot.drive.swerve.updateWithXLock() // Lock the drivetrain wheel to an X shape reduce how much we can be pushed)
                ),
                new SequentialCommandGroup(
                        new ReadyToLaunch(),
                        new ClearLaunch()
                )
        );
        addRequirements(robot.drive);
    }
}
