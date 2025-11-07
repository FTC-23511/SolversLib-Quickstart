package org.firstinspires.ftc.teamcode.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSubsystem extends SubsystemBase {

    Motor shooter1;
    Motor shooter2;
    MotorGroup shooter;
    public double getTargetVelocity() {
        return targetVelocity;
    }
    public double getActualVelocity() {
        return shooter.getCorrectedVelocity();
    }
    public double targetVelocity = 0.0;
    double kPOriginal = 0.0015;
    double kFOriginal = 0.00061;
    double kP = kPOriginal;
    double kF = kFOriginal;
    private final PIDFController flywheelController = new PIDFController(kPOriginal, 0, 0, kFOriginal);

    InterpLUT lut = new InterpLUT();


    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.RPM_312);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.RPM_312);

        shooter1.setInverted(true); //one has to be backwards
        shooter2.setInverted(false);

        shooter = new MotorGroup(shooter1, shooter2);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(0.0);

        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetVelocity(double vel) {
        targetVelocity = vel;
        flywheelController.setF(kF);
        flywheelController.setP(kP);
        flywheelController.setSetPoint(targetVelocity);
    }
    public void updatePIDVoltage(double voltage) {
        kP = (voltage / 12) * kPOriginal;
        kF = (voltage / 12) * kFOriginal;
    }

    public void periodic() {
        shooter.set(flywheelController.calculate(shooter.getCorrectedVelocity()));
    }

}
