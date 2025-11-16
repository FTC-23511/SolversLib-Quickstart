package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

/**
 * @author michael
 *
 */
public class ScanAndUpdateBallsCommand extends CommandBase {
    private ColorSensorsSubsystem colorSensorsSubsystem;
    private SpindexerSubsystem spindexerSubsystem;
    public ScanAndUpdateBallsCommand(SpindexerSubsystem spindexerSubsystem, ColorSensorsSubsystem colorSensorsSubsystem) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.colorSensorsSubsystem = colorSensorsSubsystem;
        addRequirements(colorSensorsSubsystem, spindexerSubsystem);
    }

    @Override
    public void initialize() {
        float[] colorsReadSensor1 = colorSensorsSubsystem.senseColorsHSV(1);
//        float[] colorsReadSensor2 = colorSensorsSubsystem.senseColorsHSV(2);
        RobotConstants.BallColors ball1 = ColorSensorsSubsystem.colorsHSVToBallsColors(colorsReadSensor1);
//        RobotConstants.BallColors ball2 = ColorSensorsSubsystem.colorsHSVToBallsColors(colorsReadSensor2);
        spindexerSubsystem.setBalls(
                new RobotConstants.BallColors[]{
                        ball1,
                        spindexerSubsystem.getBalls()[1],
                        spindexerSubsystem.getBalls()[2]
                }
        );
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
