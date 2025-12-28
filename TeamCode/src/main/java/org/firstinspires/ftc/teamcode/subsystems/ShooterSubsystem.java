package org.firstinspires.ftc.teamcode.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSubsystem extends SubsystemBase {
    //Note: I changed the motor type from 312 rpm to bare (6k i think).
    // We might have to redo PID and find new velocities.
    //Delete this once done :)
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
    public boolean isAtTargetVelocity() {
        return Math.abs(flywheelController.getSetPoint() - shooter1.getCorrectedVelocity()) < 50;
    }
    double kPOriginal = -0.008;
    double kFOriginal = -0.00052;
    double kP = kPOriginal;
    double kF = kFOriginal;
    InterpLUT lut;
    private final PIDFController flywheelController = new PIDFController(kPOriginal, 0, 0, kFOriginal);
    public ShooterSubsystem(final HardwareMap hMap) {
        shooter1 = new Motor(hMap, "shooter1", Motor.GoBILDA.BARE);
        shooter2 = new Motor(hMap, "shooter2", Motor.GoBILDA.BARE);
        hood = new ServoEx(hMap, "pivot");

        shooter1.setInverted(true); //one has to be backwards
        shooter2.setInverted(false);
        hood.setInverted(false);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        
        shooter = new MotorGroup(shooter1, shooter2);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.set(0);
        //Note: The distance measured is from the robot center to the spot where the ball lands in the corner, NOT the apriltag.
        lut = new InterpLUT(); //distance (in), linear speed (in/s);
        lut.add(10, 50); //placeholder
        lut.add(10, 50); //placeholder
        lut.createLUT();
    }
    public void setTargetLinearSpeed(double vel) {
        double ticksPerRev = 28.0;
        double flywheelDiameter = 2.83465;
        double gearRatio = 32.0 / 24.0; //
        double flywheelRPS = vel / (Math.PI * flywheelDiameter);
        double motorRPS = flywheelRPS / gearRatio;
        double targetTicksPerSec = motorRPS * ticksPerRev;
        flywheelController.setSetPoint(targetTicksPerSec);
    }

    /**
     *
     * @return Linear speed of flywheel in in/s
     */
    public double getFlywheelLinearSpeed() {
        double ticksPerRev = 28.0;
        double flywheelDiameter = 2.83465; //72 mm to inches
        double gearRatio = 32.0 / 24.0;
        return shooter1.getCorrectedVelocity() / ticksPerRev * gearRatio * Math.PI * flywheelDiameter;
    }
    public double findSpeedFromDistance(double distance) {
        return lut.get(distance);
    }
    public void updatePIDVoltage(double voltage) {
        double compensation = 13.5 / voltage; //if voltage < 13.5, compensation > 1
        kP = compensation * kPOriginal;
        kF = compensation * kFOriginal;
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
    @Override
    public void periodic() {
        flywheelController.setF(kF);
        flywheelController.setP(kP);
        hood.set(hoodPos);
        shooter.set(flywheelController.calculate(flywheelController.getSetPoint() != 0 ? shooter1.getCorrectedVelocity() : 0));
    }

}
