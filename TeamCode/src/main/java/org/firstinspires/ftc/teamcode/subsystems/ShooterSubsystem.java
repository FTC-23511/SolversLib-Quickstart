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
    ServoEx pivot;

    public double MIN_TICKS = 0.0;
    public double MAX_TICKS = 1.0;
    public double OFFSET = 0.0; //set the ticks to make 0 degrees
    public double pivotCurrentPos = MIN_TICKS + OFFSET;
    //degrees change on pivot bar (silver gear) convert to gold gear ticks: (6*degrees)/(1675)
    public double degreesToTicks(double degrees) {
        return ((6 * degrees) / 1675);
    }
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
        flywheelController.setSetPoint(vel);
    }
    public void updatePIDVoltage(double voltage) {
        kP = (voltage / 13.5) * kPOriginal;
        kF = (voltage / 13.5) * kFOriginal;
    }

    public void increasePivotPosition(double degrees) {
        if (pivotCurrentPos + degreesToTicks(degrees) <= MAX_TICKS) {
            pivotCurrentPos += degreesToTicks(degrees);
            pivot.set(pivotCurrentPos);
        }
        else{
            pivot.set(pivotCurrentPos);
        }
    }

    public void decreasePivotPosition(double degrees) {
        if (pivotCurrentPos - degreesToTicks(degrees) >= MIN_TICKS) {
            pivotCurrentPos -= degreesToTicks(degrees);
            pivot.set(pivotCurrentPos);
        }
        else{
            pivot.set(pivotCurrentPos);
        }
    }

    public void setPivotPosition(double pivotDegrees) {
        pivot.set(degreesToTicks(pivotDegrees)+OFFSET);
    }

    public double getPivotPosition() {
        return pivot.getRawPosition();
    }

    public void periodic() {
        flywheelController.setF(kF);
        flywheelController.setP(kP);
        shooter.set(flywheelController.calculate(shooter.getCorrectedVelocity()));
    }

}
