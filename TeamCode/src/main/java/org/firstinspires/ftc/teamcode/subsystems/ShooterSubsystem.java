package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSubsystem extends SubsystemBase {
    Motor shooter;
    public double getTargetVelocity() {
        return flywheelController.getSetPoint();
    }
    public double getActualVelocity() {
        return shooter.getCorrectedVelocity();
    }
    double kPOriginal = 0.0100;
    double kFOriginal = 0.00061;
    double kP = kPOriginal;
    double kF = kFOriginal;
    private final PIDFController flywheelController = new PIDFController(kPOriginal, 0, 0, kFOriginal);

    InterpLUT lut = new InterpLUT();

    public ShooterSubsystem(final HardwareMap hMap) {
        shooter = new Motor(hMap, "shooter", Motor.GoBILDA.RPM_312);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(0.0);

        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        //lut.add(51,1000);
        //lut.add(51,1000);
        //lut.createLUT();
    }

//    public void setTargetFromDistance(double distance) {
//        targetVelocity = (lut.get(distance));
//    }

    public void setTargetVelocity(double vel) {
        flywheelController.setSetPoint(vel);
    }
    public void updatePIDVoltage(double voltage) {
        kP = (voltage / 13.5) * kPOriginal;
        kF = (voltage / 13.5) * kFOriginal;
    }

    public void periodic() {
        flywheelController.setF(kF);
        flywheelController.setP(kP);
        shooter.set(flywheelController.calculate(shooter.getCorrectedVelocity()));
    }

}
