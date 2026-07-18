package Subsistemas;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
@Configurable
public class LauncherSub extends SubsystemBase {

    // PID + Feedforward
    private final static double KP = 0.0085;
    private final static double KV = 0.000455;
    private final PController flywheelController = new PController(KP);
    // Hardware
    private final MotorGroup flywheelMotors;
    private final MotorEx motor1;
    private final MotorEx motor2;
    private final ServoEx servoHood;
    private static final PwmControl.PwmRange HOOD_RANGE = new PwmControl.PwmRange(500, 2500);
    private static final double HOOD_MIN_ANGLE = 22.5;
    private static final double HOOD_MAX_ANGLE = 62.74;
    // Estados
    private boolean SHOOTER_RUNNING = false;
    private double MOTOR_POWER = 0.0;
    private double MOTOR_TICKS_PER_SEC;
    private double GOAL_DISTANCE;
    private double HOOD_ANGLE;
    private final InterpLUT ANGLE_LUT = new InterpLUT();
    public static double TARGET_TICKS_PER_SEC = 1300;

    public LauncherSub(HardwareMap Hm, String shooterMotor1, String shooterMotor2, String ServoHood) {
        motor1 = new MotorEx(Hm, shooterMotor1).setCachingTolerance(0.001);
        motor2 = new MotorEx(Hm, shooterMotor2).setCachingTolerance(0.001);

        flywheelMotors = new MotorGroup(motor1.setInverted(true), motor2);
        flywheelMotors.setRunMode(Motor.RunMode.RawPower);
        flywheelMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        servoHood = new ServoEx(Hm, ServoHood, 22.5, 62.74).setPwm(HOOD_RANGE);
        servoHood.setInverted(false);

        // Hood LUT
        ANGLE_LUT.add(30.0, 22.5);
        ANGLE_LUT.add(60.0, 30.0);
        ANGLE_LUT.add(70.0, 31.0);
        ANGLE_LUT.add(80.0, 32.0);
        ANGLE_LUT.add(90.0, 33.0);
        ANGLE_LUT.add(100.0, 34.0);
        ANGLE_LUT.add(110.0, 35.0);
        ANGLE_LUT.createLUT();
    }

    @Override
    public void periodic() {
        HOOD_ANGLE = ANGLE_LUT.get(GOAL_DISTANCE);
        setHOOD_ANGLE(HOOD_ANGLE);

        MOTOR_TICKS_PER_SEC = motor2.getCorrectedVelocity();

        if (!SHOOTER_RUNNING) {
            flywheelMotors.set(0.0);
            MOTOR_POWER = 0.0;
            return;
        }

        flywheelController.setSetPoint(TARGET_TICKS_PER_SEC);
        MOTOR_POWER = Math.max(-1.0,Math.min(1.0, flywheelController.calculate(MOTOR_TICKS_PER_SEC) + (KV * TARGET_TICKS_PER_SEC)));
        flywheelMotors.set(MOTOR_POWER);
    }

    public void setGOAL_DISTANCE(double distance) {
        GOAL_DISTANCE = distance;
    }
    public void setHOOD_ANGLE(double angle) {
        servoHood.set(Math.max(HOOD_MIN_ANGLE, Math.min(HOOD_MAX_ANGLE, angle)));
    }
    public void startShooter() {
        SHOOTER_RUNNING = true;
        flywheelController.reset();
    }
    public void stopShooter() {
        SHOOTER_RUNNING = false;
        flywheelMotors.set(0.0);
    }
    public void toggleShooter() {
        if (SHOOTER_RUNNING) stopShooter();
        else startShooter();
    }
    // Getters
    public double getTicksPerSec() {
        return MOTOR_TICKS_PER_SEC;
    }
    public double getTicksPerSecError() {
        return TARGET_TICKS_PER_SEC - MOTOR_TICKS_PER_SEC;
    }
}