package Subsistemas;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Configurable
public class LauncherSub extends SubsystemBase {

    // PID + Feedforward
    private final PIDFController flyWheelPDIFController = new PIDFController(kP, kI, kD, 0.0);
    private SimpleMotorFeedforward flyWheelFController;

    // Hardware
    private final MotorGroup flywheelMotors;
    private final MotorEx motor1;
    private final MotorEx motor2;
    private final ServoEx servoHood;

    private final PwmControl.PwmRange HoodRange = new PwmControl.PwmRange(500, 2500);
    private static final double HOOD_MIN_ANGLE = 22.5;
    private static final double HOOD_MAX_ANGLE = 62.74;

    // Estados
    private boolean shooterRunning = false;
    private double flywheelPower = 0.0;
    private double lastKs = Ks;
    private double lastKv = Kv;
    private double motorRPM;
    private double goalDistance;
    private double hoodAngle;

    // Configurables
    private final InterpLUT hoodLUT = new InterpLUT();

    public static double targetRPM = 2000;
    public static double kP = 0.015;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double Ks = 0.31; 
    public static double Kv = 0.000285;
    public static double Ka = 0.00005;   // ← Agregado y con valor inicial recomendado

    public LauncherSub(HardwareMap Hm, String shooterMotor1, String shooterMotor2, String ServoHood) {
        motor1 = new MotorEx(Hm, shooterMotor1).setCachingTolerance(0.001);
        motor2 = new MotorEx(Hm, shooterMotor2).setCachingTolerance(0.001);

        flywheelMotors = new MotorGroup(motor1.setInverted(true), motor2);

        servoHood = new ServoEx(Hm, ServoHood, 22.5, 62.74).setPwm(HoodRange);
        servoHood.setInverted(false);

        flywheelMotors.setRunMode(Motor.RunMode.RawPower);
        flywheelMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flyWheelPDIFController.setTolerance(14);

        // Inicializar Feedforward con Ka
        flyWheelFController = new SimpleMotorFeedforward(Ks, Kv, Ka);

        // Hood LUT
        hoodLUT.add(30.0,22.5);
        hoodLUT.add(60.0,30.0);
        hoodLUT.add(70.0,31.0);
        hoodLUT.add(80.0,32.0);
        hoodLUT.add(90.0,33.0);
        hoodLUT.add(100.0,34.0);
        hoodLUT.add(110.0,35.0);
        hoodLUT.createLUT();
    }

    @Override
    public void periodic() {
        hoodAngle = hoodLUT.get(goalDistance);
        setHoodAngle(hoodAngle);

        motorRPM = motor2.getCorrectedVelocity();

        if (!shooterRunning) {
            flywheelMotors.set(0.0);
            flywheelPower = 0.0;
            return;
        }

        flyWheelPDIFController.setPIDF(kP, kI, kD, 0);

        // Actualizar Feedforward si cambian los valores
        if (lastKs != Ks || lastKv != Kv) {
            flyWheelFController = new SimpleMotorFeedforward(Ks, Kv, Ka);
            lastKs = Ks;
            lastKv = Kv;
        }

        flyWheelPDIFController.setSetPoint(targetRPM);
        flywheelPower = flyWheelFController.calculate(targetRPM);
        flywheelPower += flyWheelPDIFController.calculate(motorRPM);

        flywheelMotors.set(Math.max(-1.0, Math.min(1.0, flywheelPower)));
    }

    public void setGoalDistance(double distance) { goalDistance = distance; }

    public void setHoodAngle(double angle) {
        servoHood.set(Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, angle)));
    }

    public void startShooter() {
        shooterRunning = true;
        flyWheelPDIFController.reset();
    }

    public void stopShooter() {
        shooterRunning = false;
        flywheelMotors.set(0.0);
    }

    public void toggleShooter() {
        if (shooterRunning) stopShooter();
        else startShooter();
    }

    // Getters
    public boolean isShooterRunning() { return shooterRunning; }
    public double getMotorRPM() { return motorRPM; }
    public double getTargetRPM() { return targetRPM; }
    public double getRPMError() { return targetRPM - motorRPM; }
    public double getFlywheelPower() { return flywheelPower; }
    public double getHoodAngle() { return hoodAngle; }
}