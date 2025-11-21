package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootBallSequenceCommandSequence extends SequentialCommandGroup {
    public ShootBallSequenceCommandSequence(ShooterSubsystem shooterSubsystem, SpindexerSubsystem spindexerSubsystem, GateSubsystem gateSubsystem, RobotConstants.BallColors[] targetBallSequence) {
        addCommands(
                new InstantCommand(gateSubsystem::down), //redundant, for safety b/c gate should already be up.
                //ball 1 logic
                new ConditionalCommand( //is targetBallSequence[0] loaded?
                        new SequentialCommandGroup( //onTrue, gate down
                                new InstantCommand(gateSubsystem::down),
                                new WaitCommand(300)
                        ),
                        new SequentialCommandGroup( //onFalse, load targetBallSequence[0] then gate down
                                new LoadBallCommand(spindexerSubsystem, targetBallSequence[0]),
                                new InstantCommand(gateSubsystem::down),
                                new WaitCommand(300)
                        ),
                        () -> spindexerSubsystem.getBalls()[2] == targetBallSequence[0] || targetBallSequence[0] == RobotConstants.BallColors.UNKNOWN //ball is already loaded or we don't care . condition as booleanSupplier
                ),
                new MoveSpindexerCommand(spindexerSubsystem, gateSubsystem, 1, false), //shoot ball
                //ball 2 logic is a bit different- we don't want to move gate up if a ball is already loaded
                new ConditionalCommand( //is targetBallSequence[1] loaded?
                        new InstantCommand(), //onTrue, do nothing as gate is already down
                        new SequentialCommandGroup( //onFalse, gate up -> load target -> gate down
//                                new InstantCommand(gateSubsystem::up),
//                                new WaitCommand(300),
                                new LoadBallCommand(spindexerSubsystem, targetBallSequence[1])
//                                new InstantCommand(gateSubsystem::down),
//                                new WaitCommand(300)
                        ),
                        () -> spindexerSubsystem.getBalls()[2] == targetBallSequence[1] || targetBallSequence[1] == RobotConstants.BallColors.UNKNOWN //ball is already loaded or we don't care . condition as booleanSupplier
                ),
                new MoveSpindexerCommand(spindexerSubsystem, gateSubsystem, 1, false), //shoot ball
                //ball 3 logic
                new ConditionalCommand( //is targetBallSequence[2] loaded?
                    new InstantCommand(), //onTrue, do nothing as gate is already down
                    new SequentialCommandGroup( //onFalse, gate up -> load final ball -> gate down
//                        new InstantCommand(gateSubsystem::up),
//                        new WaitCommand(300),
                        new LoadBallCommand(spindexerSubsystem, targetBallSequence[2])
//                        new InstantCommand(gateSubsystem::down),
//                        new WaitCommand(300)
                    ),
                    () -> spindexerSubsystem.getBalls()[2] == targetBallSequence[2] || targetBallSequence[2] == RobotConstants.BallColors.UNKNOWN //ball is already loaded or we don't care . condition as booleanSupplier
                ),
                new MoveSpindexerCommand(spindexerSubsystem, gateSubsystem, 1, false), //shoot ball
                new InstantCommand(gateSubsystem::down)
        );
        addRequirements(shooterSubsystem, spindexerSubsystem, gateSubsystem);
    }

}
