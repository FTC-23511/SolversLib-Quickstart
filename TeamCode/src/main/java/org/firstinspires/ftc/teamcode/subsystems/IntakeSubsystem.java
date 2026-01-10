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
    private CRServo sideWheel;
    private CRServo sideWheel2;
    public IntakeSubsystem(final HardwareMap hMap) {
        intakeWheels = hMap.get(DcMotor.class, "intake");
        intakeWheels.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sideWheel = hMap.get(CRServo.class, "intakeSide1");
        sideWheel2 = hMap.get(CRServo.class, "intakeSide2");
    }

    public void set(IntakeState state) {
        switch(state) {
            case STILL:
                intakeWheels.setPower(0.0);
                sideWheel.setPower(0.0);
                sideWheel2.setPower(0.0);
                break;
            case INTAKING:
                intakeWheels.setPower(1.0);
                sideWheel.setPower(1.0);
                sideWheel2.setPower(-1.0);
                break;
            case REVERSE:
                intakeWheels.setPower(-1.0);
                sideWheel.setPower(-1.0);
                sideWheel2.setPower(1.0);
                break;
        }
        intakeState = state;
    }
}