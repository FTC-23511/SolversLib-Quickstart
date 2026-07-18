package Subsistemas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
public class TurretSubsystem_Autonomous extends SubsystemBase {
    private final DcMotorEx turret;
    private static final double POWER = 0.5;
    private static final int TOLERANCE = 10; // ticks

    public TurretSubsystem_Autonomous(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(int position) {
        turret.setTargetPosition(position);
        turret.setPower(POWER);
    }

    public boolean isAtTarget() {
        return !turret.isBusy() ||
                Math.abs(turret.getCurrentPosition() - turret.getTargetPosition()) < TOLERANCE;
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public void stop() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}