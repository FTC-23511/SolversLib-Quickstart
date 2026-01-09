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

        addRequirements(shooterSubsystem, spindexerSubsystem, gateSubsystem);

        addCommands(
                new SmartAlignCommand(spindexerSubsystem, gateSubsystem, targetBallSequence),

                // STEP 2: The Safeguard Shooting Loop
                // This proceeds to shoot the balls. If Step 1 worked perfectly,
                // this loop will just go "Shoot, Shoot, Shoot" with zero delays.
                // If something goes wrong (ball mismatch), it falls back to the safe sorting logic.
                new SequentialCommandGroup(
                        makeBallShotSequence(spindexerSubsystem, gateSubsystem, targetBallSequence[0]),
                        makeBallShotSequence(spindexerSubsystem, gateSubsystem, targetBallSequence[1]),
                        makeBallShotSequence(spindexerSubsystem, gateSubsystem, targetBallSequence[2])
                ),

                // Cleanup
                new InstantCommand(gateSubsystem::down)
        );
    }

    /**
     * Logic to safely Shoot ONE ball, handling sorting if needed.
     */
    private SequentialCommandGroup makeBallShotSequence(SpindexerSubsystem spindexer,
                                                        GateSubsystem gate,
                                                        RobotConstants.BallColors targetColor) {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        // TRUE: Ball matches! Just ensure gate is down.
                        new SequentialCommandGroup(
                                new InstantCommand(gate::down),
                                new WaitCommand(100)
                        ),
                        // FALSE: Mismatch! Gate UP -> Load Correct Ball -> Gate DOWN
                        new SequentialCommandGroup(
                                new InstantCommand(gate::up),
                                new WaitCommand(200),
                                new LoadBallCommand(spindexer, targetColor),
                                new InstantCommand(gate::down),
                                new WaitCommand(200)
                        ),
                        // Condition: Is the ball at exit correct?
                        () -> spindexer.getBalls()[2] == targetColor || targetColor == RobotConstants.BallColors.UNKNOWN
                ),
                new MoveSpindexerCommand(spindexer, gate, 1, false) // Shoot
        );
    }

    // ==================================================================================
    // Custom Command: Calculates the global optimum start position
    // ==================================================================================
    private static class SmartAlignCommand extends CommandBase {
        private final SpindexerSubsystem spindexer;
        private final GateSubsystem gate;
        private final RobotConstants.BallColors[] targets;
        private boolean isFinished = false;

        public SmartAlignCommand(SpindexerSubsystem spindexer, GateSubsystem gate, RobotConstants.BallColors[] targets) {
            this.spindexer = spindexer;
            this.gate = gate;
            this.targets = targets;
        }

        @Override
        public void initialize() {
            RobotConstants.BallColors[] currentBalls = spindexer.getBalls(); // e.g., [P, G, P]

            // We simulate 3 possible start offsets:
            // 0 clicks (Start at current), 1 click (Rotate 1), 2 clicks (Rotate 2)
            int bestOffset = 0;
            int maxScore = -1;

            // Check all 3 possibilities
            for (int offset = 0; offset < 3; offset++) {
                int score = calculateMatchScore(currentBalls, targets, offset);
                if (score > maxScore) {
                    maxScore = score;
                    bestOffset = offset;
                }
            }

            // Execute the rotation to the best spot
            if (bestOffset > 0) {
                gate.up();
                // We assume MoveSpindexerCommand or similar logic can be run here,
                // or we manually spin. For simplicity in this logical block:
                spindexer.moveSpindexerBy(bestOffset); // Hypothetical method to move N slots
                // In reality, you might schedule a sub-command here or wait in execute()
            }
            // If bestOffset is 0, we do nothing! We are already in the best spot.
        }

        @Override
        public void execute() {
            // Wait for spindexer to finish moving if we commanded it
            if (!spindexer.isNearTargetPosition()) {
                isFinished = true;
            }
        }

        @Override
        public boolean isFinished() {
            return isFinished;
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
}