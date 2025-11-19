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
    private GateSubsystem gateSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    private Runnable action;

    private double[] targetPositions = {60.0, 180.0, 300.0};
    private double tolerance = 5.0;
    private boolean actionHasRan;

    public ScheduleGateCommand(SpindexerSubsystem spindexerSubsystem, GateSubsystem gateSubsystem, Runnable action) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.gateSubsystem = gateSubsystem;
        this.action = action;
        actionHasRan = false;
    }

    @Override
    public void execute() {
        for (double targetPosition : targetPositions) {
            if (Math.abs(spindexerSubsystem.getCurrentPosition() - targetPosition) < tolerance) {
                action.run();
                actionHasRan = true;
            }
        }
    }
    @Override
    public boolean isFinished() {
        return actionHasRan;
    }
}
