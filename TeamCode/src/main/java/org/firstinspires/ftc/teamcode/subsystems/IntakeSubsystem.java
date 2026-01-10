package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        STILL, INTAKING, REVERSE;
    }
    public IntakeState intakeState = IntakeState.STILL;
    private DcMotor intakeWheels;
    private CRServoEx sideWheel;
    private CRServoEx sideWheel2;
    public IntakeSubsystem(final HardwareMap hMap) {
        intakeWheels = hMap.get(DcMotor.class, "intake");
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sideWheel = hMap.get(CRServoEx.class, "intakeSide1");
        sideWheel2 = hMap.get(CRServoEx.class, "intakeSide2");
    }

    public void set(IntakeState state) {
        switch(state) {
            case STILL:
                intakeWheels.setPower(0.0);
                sideWheel.set(0.0);
                sideWheel2.set(0.0);
                break;
            case INTAKING:
                intakeWheels.setPower(1.0);
                sideWheel.set(1.0);
                sideWheel2.set(-1.0);
                break;
            case REVERSE:
                intakeWheels.setPower(-1.0);
                sideWheel.set(-1.0);
                sideWheel2.set(1.0);
                break;
        }
        intakeState = state;
    }
}