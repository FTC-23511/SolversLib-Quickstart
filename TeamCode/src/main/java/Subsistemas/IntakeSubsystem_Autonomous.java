package Subsistemas;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
public class IntakeSubsystem_Autonomous extends SubsystemBase {
    private final DcMotorEx intake;
    public IntakeSubsystem_Autonomous(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Transfer");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    public void setPower(double Velocity) {
        intake.setVelocity(Velocity);
    }
    public void stop() {
        intake.setVelocity(0);
    }
}