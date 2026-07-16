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
    // ================== PID/FF ===================
    private final PIDFController flyWheelPDIFController = new PIDFController(kP, kI, kD, 0.0);
    private SimpleMotorFeedforward flyWheelFController = new SimpleMotorFeedforward(Ks, Kv);

    // ================= HARDWARE =================
    private final MotorGroup flywheelMotors;
    private final MotorEx motor1;
    private final MotorEx motor2;
    private final ServoEx servoHood;

    private final PwmControl.PwmRange HoodRange = new PwmControl.PwmRange(500, 2500);
    private static final double HOOD_MIN_ANGLE = 22.5;
    private static final double HOOD_MAX_ANGLE = 62.74;

    // ================= ESTADOS =================
    private boolean shooterRunning = false;
    private double flywheelPower = 0.0;
    private double lastKs = Ks;
    private double lastKv = Kv;
    private double motorRPM;
    private double motorAcl;
    private double goalDistance;
    private double hoodAngle;

    // ================= CONFIGURABLES =================
    private final InterpLUT hoodLUT = new InterpLUT();

    public static double targetRPM = 2000;
    public static double targeAccerelation = 40;
    public static double kP = 0.015;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double Ks = 0.28;
    public static double Kv = 0.00028;
    public static double Ka = 0.0;

    public LauncherSub(HardwareMap Hm, String shooterMotor1, String shooterMotor2, String ServoHood) {
        motor1 = new MotorEx(Hm, shooterMotor1).setCachingTolerance(0.001);
        motor2 = new MotorEx(Hm, shooterMotor2).setCachingTolerance(0.001);

        flywheelMotors = new MotorGroup(
                motor1.setInverted(true),
                motor2
        );

        servoHood = new ServoEx(Hm, ServoHood, 22.5, 62.74).setPwm(HoodRange);
        servoHood.setInverted(false);

        flywheelMotors.setRunMode(Motor.RunMode.RawPower);
        flywheelMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flyWheelPDIFController.setTolerance(5);

        // Hood LUT
        hoodLUT.add(30.0, 22.5);
        hoodLUT.add(60.0, 30.0);
        hoodLUT.add(70.0, 31.0);
        hoodLUT.add(80.0, 32.0);
        hoodLUT.add(90.0, 33.0);
        hoodLUT.add(100.0, 34.0);
        hoodLUT.add(110.0, 35.0);
        hoodLUT.createLUT();
    }

    @Override
    public void periodic() {
        hoodAngle = hoodLUT.get(goalDistance);
        setHoodAngle(hoodAngle);

        motorRPM = motor2.getCorrectedVelocity();
        motorAcl = motor2.getAcceleration();

        if (!shooterRunning) {
            flywheelMotors.stopMotor();
            flywheelPower = 0.0;
            return;
        }

        flyWheelPDIFController.setPIDF(kP, kI, kD, 0);

        if (lastKs != Ks || lastKv != Kv) {
            flyWheelFController = new SimpleMotorFeedforward(Ks, Kv, Ka);
            lastKs = Ks;
            lastKv = Kv;
        }

        flyWheelPDIFController.setSetPoint(targetRPM);
        flywheelPower = flyWheelFController.calculate(targetRPM);
        flywheelPower += flyWheelPDIFController.calculate(motorRPM);

        flywheelMotors.set(clamp(flywheelPower, -1.0, 1.0));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void setGoalDistance(double distance) {
        goalDistance = distance;
    }

    public void setHoodAngle(double angle) {
        servoHood.set(clamp(angle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE));
    }

    public void startShooter() {
        shooterRunning = true;
        flyWheelPDIFController.reset();
    }

    public void stopShooter() {
        shooterRunning = false;
        flywheelMotors.stopMotor();
    }

    public void toggleShooter() {
        if (shooterRunning) {
            stopShooter();
        } else {
            startShooter();
        }
    }


    // ================= TELEMETRY GETTERS =================

    public double getMotorRPM() {
        return motorRPM;
    }

    public double motorpower() {return motor2.getRawPower();}

    public double getShooterAcl(){return motorAcl;}
    public double getShooterTargetAcl(){return targeAccerelation;}
    public double getAclError(){return targeAccerelation - motorAcl;}



    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPMError() {
        return targetRPM - motorRPM;
    }

    public double getFlywheelPower() {
        return flywheelPower;
    }

    public double getDistanceToGoal() {
        return goalDistance;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }

    public boolean isShooterRunning() {
        return shooterRunning;
    }

    public double getHoodServoPosition() {
        return servoHood.get();
    }

    public double getHoodServoRawPosition() {
        return servoHood.getRawPosition();
    }

}
