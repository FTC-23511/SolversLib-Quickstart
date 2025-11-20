package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

/**
 * @author michael
 * @class "schedules" the gate to move either up or down, only when spindexer.getCurrentAnalogPosition() is within 5 degrees of the 60deg position.
 */
public class ScheduleGateCommand extends CommandBase {
    private final SpindexerSubsystem spindexer;
    private final Runnable action;

    private final double[] targets = {60, 180, 300};
    private final double tolerance = 1;  // bigger tolerance ALWAYS fixes jumpiness

    private boolean triggered = false;
    private double prevPos;

    public ScheduleGateCommand(SpindexerSubsystem spindexer, GateSubsystem gate, Runnable action) {
        this.spindexer = spindexer;
        this.action = action;

    }

    @Override
    public void initialize() {
        prevPos = spindexer.getWrappedPosition();
        triggered = false;
    }

    @Override
    public void execute() {
        double pos = spindexer.getWrappedPosition();

        for (double target : targets) {

            boolean wasOutside = Math.abs(prevPos - target) > tolerance;
            boolean nowInside = Math.abs(pos - target) <= tolerance;

            // Only trigger when entering the band, not just being in it
            if (wasOutside && nowInside && !triggered) {
                action.run();
                triggered = true;
                break;
            }
        }

        prevPos = spindexer.getCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        return triggered;
    }
}
