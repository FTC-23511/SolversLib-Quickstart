package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.Arrays;

public class ShootSortedBallsCommandSequence extends SequentialCommandGroup {

    public ShootSortedBallsCommandSequence(ShooterSubsystem shooterSubsystem,
                                           SpindexerSubsystem spindexerSubsystem,
                                           GateSubsystem gateSubsystem,
                                           RobotConstants.BallColors[] targetBallSequence) {

        //1. Choose the best spindexer offset.
        RobotConstants.BallColors[] current = spindexerSubsystem.getBalls();
        int bestOffset = 0;
        int bestScore = 0;
        for (int offset = 0; offset < 3; offset++) {
            int score = calculateMatchScore(current, targetBallSequence, offset);
            if (score > bestScore) {
                bestScore = score;
                bestOffset = offset;
            }
        }
        if (bestScore == 2) {
            bestScore = 3;
        }
        //2a. Move to the best offset
        if (bestOffset > 0) {
            addCommands(
                    new InstantCommand(gateSubsystem::up),
                    new WaitForGateCommand(gateSubsystem),
                    new MoveSpindexerCommand(spindexerSubsystem, gateSubsystem, bestOffset, false),
                    new InstantCommand(gateSubsystem::down)
            );
        } else {
            addCommands(new InstantCommand(gateSubsystem::down), new WaitForGateCommand(gateSubsystem));
        }
        //2b. ...and shoot the sequence.
        addCommands(new MoveSpindexerCommand(spindexerSubsystem, gateSubsystem, bestScore, false));

        //3. Shoot the remaining balls
        for (int i = bestScore; i < 3; i++) {
            RobotConstants.BallColors target = targetBallSequence[i];
            boolean isLastBall = (i == 2);

            if (isLastBall) {
                // If it's the very last ball, we can't sort it (no other balls to swap).
                // Just ensure gate is down and shoot.
                addCommands(new InstantCommand(gateSubsystem::down));
            } else {
                // If we still have choices (2 balls left), we try to sort.
                addCommands(new ConditionalCommand(
                        // If true, ball matches. Keep gate down.
                        new InstantCommand(gateSubsystem::down),

                        // If false: open gate -> load correct ball -> close gate
                        new SequentialCommandGroup(
                                new InstantCommand(gateSubsystem::up),
                                new WaitForGateCommand(gateSubsystem),
                                new LoadBallCommand(spindexerSubsystem, target),
                                new InstantCommand(gateSubsystem::down),
                                new WaitForGateCommand(gateSubsystem)
                        ),
                        // Check if the ball at exit matches target
                        () -> spindexerSubsystem.getBalls()[2] == target || target == RobotConstants.BallColors.UNKNOWN
                ));
            }

            // Shoot the ball
            addCommands(new MoveSpindexerCommand(spindexerSubsystem, gateSubsystem, 1, false));
        }

        // Move gate back up
        addCommands(new InstantCommand(gateSubsystem::up));
    }
    /**
     * Helper: How many balls match the target sequence if we start at 'offset'?
     * Spindexer Order Assumption: Exit is Index 2. Order of arrival is 2 -> 1 -> 0.
     */
    private int calculateMatchScore(RobotConstants.BallColors[] balls,
                                    RobotConstants.BallColors[] targets,
                                    int startOffset) {
        int score = 0;
        // Spindexer indices are usually 0, 1, 2.
        // If we rotate 'startOffset' times, the effective start index shifts.
        // Let's assume standard rotation:
        // Offset 0: Sequence is balls[2], balls[1], balls[0]
        // Offset 1: Sequence is balls[1], balls[0], balls[2]
        // Offset 2: Sequence is balls[0], balls[2], balls[1]

        int[] sequenceIndices = {2, 1, 0};

        for (int i = 0; i < 3; i++) {
            // Calculate which ball slot is at the exit for step 'i'
            // given our initial 'startOffset'
            int currentIndexPointer = (startOffset + i) % 3;
            int actualSlotIndex = sequenceIndices[currentIndexPointer];

            RobotConstants.BallColors ball = balls[actualSlotIndex];
            RobotConstants.BallColors target = targets[i];

            if (ball == target || target == RobotConstants.BallColors.UNKNOWN) {
                score++;
            } else {
                // Optimization Strategy:
                // If we break the streak, we stop counting.
                // We prioritize a perfect START over a high total count.
                // e.g. Match-Match-Fail (Score 2) is better than Fail-Match-Match (Score 0)
                break;
            }
        }
        return score;
    }
}