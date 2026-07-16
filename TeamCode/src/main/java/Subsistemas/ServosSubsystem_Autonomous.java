package Subsistemas;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class ServosSubsystem_Autonomous extends SubsystemBase {
    private final ServoEx hood, tope;
    public ServosSubsystem_Autonomous(HardwareMap hardwareMap) {
        hood = new ServoEx(hardwareMap, "hood");
        tope = new ServoEx(hardwareMap, "ServoTope");
        tope.setCachingTolerance(0.001);
        hood.setCachingTolerance(0.001);

    }
    public void PositionHood(double Position) {
        hood.set(Position);
    }
    public void TopeAbierto() {
        tope.set(0);
    }

    public void TopeCerrado() {
        tope.set(0);
    }
}
