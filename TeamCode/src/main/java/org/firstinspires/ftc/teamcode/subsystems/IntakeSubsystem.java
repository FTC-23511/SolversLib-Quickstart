package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        STILL, INTAKING, REVERSE;
    }
    public IntakeState intakeState = IntakeState.STILL;
    private DcMotor intakeWheels;
    private double kP = 0.000;
    private double kI = 0.000;
    private double kD = 0.000;
    private double kF = 0.000;
    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    public IntakeSubsystem(final HardwareMap hMap) {
        intakeWheels = hMap.get(DcMotor.class, "intakeWheels");
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPIDTarget(int num) {
        pidf.setSetPoint(num);
    }

    public void setSpeed(IntakeState state) {
        switch(state) {
            case STILL:
                setPIDTarget(0);
                break;
            case INTAKING:
                setPIDTarget(1);
                break;
            case REVERSE:
                setPIDTarget(-1);
                break;
        }
    }

    public void periodic() {
        intakeWheels.setPower(pidf.calculate(intakeWheels.getCurrentPosition()));
    }
}