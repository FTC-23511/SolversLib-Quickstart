package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class LoadMotifCommand extends CommandBase {
    private final SpindexerSubsystem spindexerSubsystem;
    private final RobotConstants.BallColors[] motif;
    private boolean alreadyAligned = false;

    public LoadMotifCommand(SpindexerSubsystem spindexerSubsystem, RobotConstants.BallColors[] motif) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.motif = motif;
        addRequirements(spindexerSubsystem);
    }

    @Override
    public void initialize() {
        RobotConstants.BallColors[] balls = spindexerSubsystem.getBalls();

        int bestOffset = 0;
        int bestScore = -1;

        // Test all 3 possible spindexer rotations: 0, 1, or 2 shifts
        for (int offset = 0; offset < 3; offset++) {
            int score = 0;

            // 1st Shot: motif[0] needs to be at slot 2 (the exit).
            // Calculate which original ball index will end up at slot 2 after 'offset' shifts.
            int slotForFirstShot = (2 - offset + 3) % 3;
            if (balls[slotForFirstShot] == motif[0] || motif[0] == RobotConstants.BallColors.UNKNOWN) {
                score++;

                // 2nd Shot: motif[1] needs to be at slot 1.
                int slotForSecondShot = (1 - offset + 3) % 3;
                if (balls[slotForSecondShot] == motif[1] || motif[1] == RobotConstants.BallColors.UNKNOWN) {
                    score++;

                    // 3rd Shot: motif[2] needs to be at slot 0.
                    int slotForThirdShot = (0 - offset + 3) % 3;
                    if (balls[slotForThirdShot] == motif[2] || motif[2] == RobotConstants.BallColors.UNKNOWN) {
                        score++;
                    }
                }
            }

            // Keep the rotation that gives the longest unbroken streak
            if (score > bestScore) {
                bestScore = score;
                bestOffset = offset;
            }
        }

        // Execute the best calculated rotation
        if (bestOffset == 1) {
            spindexerSubsystem.moveSpindexerBy(120);
            spindexerSubsystem.shiftBallsArrayBy(1);
        } else if (bestOffset == 2) {
            spindexerSubsystem.moveSpindexerBy(240);
            spindexerSubsystem.shiftBallsArrayBy(2);
        } else {
            // bestOffset == 0 means the balls are already in the best possible order
            alreadyAligned = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (alreadyAligned) {
            return true;
        }
        return (spindexerSubsystem.isNearTargetPosition() && spindexerSubsystem.isLowVelocity());
    }
}