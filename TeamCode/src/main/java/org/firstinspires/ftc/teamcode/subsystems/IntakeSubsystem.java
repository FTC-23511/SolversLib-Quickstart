package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        STILL, INTAKING, REVERSE;
    }
    public IntakeState intakeState = IntakeState.STILL;
    private DcMotor intakeWheels;
    public IntakeSubsystem(final HardwareMap hMap) {
        intakeWheels = hMap.get(DcMotor.class, "intake");
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSpeed(IntakeState state) {
        switch(state) {
            case STILL:
                intakeWheels.setPower(0.0);
                break;
            case INTAKING:
                intakeWheels.setPower(1.0);
                break;
            case REVERSE:
                intakeWheels.setPower(-1.0);
                break;
        }
        intakeState = state;
    }
}