package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class SpindexerSubsystem extends SubsystemBase {
    private DcMotor spindexer;
    private double kP = 0.000;
    private double kI = 0.005;
    private double kD = 0.000;
    private double kF = 0.000;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public enum SpindexerState {ONE, TWO, THREE}
    public SpindexerState spindexerState = SpindexerState.ONE;

    private double lastOutput = -9999999;
    private double output = SPINDEXER_INITPOS;

    public SpindexerSubsystem(final HardwareMap hm) {
        spindexer = hm.get(DcMotor.class, "spindexer");
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void advanceSpindexer() {
        pidf.setSetPoint(pidf.getSetPoint() + SPINDEXER_TICKS_PER_DEG * 120);
        switch (spindexerState) {
            case ONE:
                spindexerState=SpindexerState.TWO;
                break;
            case TWO:
                spindexerState=SpindexerState.THREE;
                break;
            case THREE:
                spindexerState=SpindexerState.ONE;
                break;
        }
    }

    public void periodic() {
        lastOutput = output;
        output = pidf.calculate(spindexer.getCurrentPosition());

        if (Math.abs(lastOutput - output) < SPINDEXER_CACHETHRESHOLD) {
            spindexer.setPower(output);
        }
    }

    //kinda useless
    public void killSpindexerPower() {
        spindexer.setPower(0.0);
    }
}
