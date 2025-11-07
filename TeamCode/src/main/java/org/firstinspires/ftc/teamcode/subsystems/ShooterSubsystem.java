package org.firstinspires.ftc.teamcode.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

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
    ServoEx pivot;

    public double MIN_TICKS = 0.0;
    public double MAX_TICKS = 1.0;
    public double OFFSET = 0.5;
    public double pivotCurrentPos = MIN_TICKS + OFFSET;
    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.RPM_312);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.RPM_312);
        pivot = new ServoEx(hMap, "pivot", MIN_TICKS, MAX_TICKS);

        shooter1.setInverted(true); //one has to be backwards
        shooter2.setInverted(false);
        pivot.setInverted(false);

        shooter = new MotorGroup(shooter1, shooter2);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(MIN_TICKS);
        pivot.set(pivotCurrentPos);

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

    public void increasePivotPosition(double tickPosition) {
        if (pivotCurrentPos + tickPosition <= MAX_TICKS) {
            pivotCurrentPos += tickPosition;
            pivot.set(pivotCurrentPos);
        }
        else{
            pivot.set(pivotCurrentPos);
        }
    }

    public void decreasePivotPosition(double ticksPosition) {
        if (pivotCurrentPos - ticksPosition >= MIN_TICKS) {
            pivotCurrentPos -= ticksPosition;
            pivot.set(pivotCurrentPos);
        }
        else{
            pivot.set(pivotCurrentPos);
        }
    }

    public double getPivotPosition() {
        return pivot.getRawPosition();
    }

    public void periodic() {
        shooter.set(flywheelController.calculate(shooter.getCorrectedVelocity()));
    }

}
