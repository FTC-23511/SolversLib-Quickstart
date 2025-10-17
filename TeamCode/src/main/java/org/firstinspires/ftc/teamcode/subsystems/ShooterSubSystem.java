package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSubSystem extends SubsystemBase {


    public static double p = 0.00002, i = 0.0, d = 0.0002;
    public static double s = 500, v = 0.0005, a = 0.0;
    Motor shooter;
    public double getTargetVelocity() {
        return targetVelocity;
    }
    public double targetVelocity = 0.0;

    InterpLUT lut = new InterpLUT();

    public ShooterSubSystem(final HardwareMap hMap) {
        shooter = new Motor(hMap, "shooter", Motor.GoBILDA.RPM_312);

        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(p, i, d);
        shooter.setFeedforwardCoefficients(s, v, a);
        shooter.set(0.0);

        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);



        //lut.add(1.0,1.0);
        //lut.createLUT();
    }

    public void setTargetFromDistance(double distance) {
        targetVelocity = (lut.get(distance));
    }

    public void setTargetVelocity(double num) {
        targetVelocity = num;
        shooter.set(targetVelocity);
    }

    public void periodic() {
//        shooter.set(targetVelocity);
    }

}
