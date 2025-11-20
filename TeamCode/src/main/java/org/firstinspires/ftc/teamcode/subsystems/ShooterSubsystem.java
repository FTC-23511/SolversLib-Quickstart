package org.firstinspires.ftc.teamcode.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class ShooterSubsystem extends SubsystemBase {

    private Motor shooter1;
    private Motor shooter2;
    private ServoEx hood;
    private MotorGroup shooter;
    private double hoodPos = 0.6;
    public double getTargetVelocity() {
        return flywheelController.getSetPoint();
    }
    public double getActualVelocity() {
        return shooter1.getCorrectedVelocity();
    }
    double kPOriginal = -0.008;
    double kFOriginal = -0.00052;
    double kP = kPOriginal;
    double kF = kFOriginal;
    private final PIDFController flywheelController = new PIDFController(kPOriginal, 0, 0, kFOriginal);
    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.RPM_312);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.RPM_312);
        hood = new ServoEx(hMap, "pivot");

        shooter1.setInverted(false); //one has to be backwards
        shooter2.setInverted(true);
        hood.setInverted(false);

        shooter = new MotorGroup(shooter1, shooter2);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(0);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetVelocity(double vel) {
        flywheelController.setSetPoint(vel);
    }
    public void updatePIDVoltage(double voltage) {
        kP = (voltage / 13.5) * kPOriginal;
        kF = (voltage / 13.5) * kFOriginal;
    }
    public void setHood(double ticks) {
        hoodPos = ticks;
    }

    /**
     *
     * @return last commanded hood pos
     */
    public double getHoodPos() {
        return hoodPos;
    }

    public void periodic() {
        flywheelController.setF(kF);
        flywheelController.setP(kP);
        hood.set(hoodPos);
        shooter.set(flywheelController.calculate(shooter1.getCorrectedVelocity()));
    }

}
