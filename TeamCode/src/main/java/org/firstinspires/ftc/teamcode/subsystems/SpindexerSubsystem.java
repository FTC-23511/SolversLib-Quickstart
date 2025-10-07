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
    private double kI = 0.000;
    private double kD = 0.000;
    private double kF = 0.000;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public enum SpindexerState {ONE, TWO, THREE}
    public SpindexerState spindexerState = SpindexerState.ONE;

    private double lastOutput = -9999999;
    private double output = 0;

    public SpindexerSubsystem(final HardwareMap hm) {
        spindexer = hm.get(DcMotor.class, "spindexer");
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setSpindexerState(SpindexerState state){
        switch (state) {
            case ONE:
                setPIDTarget(1);
                break;
            case TWO:
                setPIDTarget(2);
                break;
            case THREE:
                setPIDTarget(3);
                break;
        }
        spindexerState = state;
    }
    public void setPIDTarget(int num) {
        pidf.setSetPoint(num);
    }

    public void periodic() {
        lastOutput = output;
        output = pidf.calculate(spindexer.getCurrentPosition());

        if (lastOutput - output < SPINDEXER_CACHETHRESHOLD) {
            spindexer.setPower(output);
        }
    }
}
