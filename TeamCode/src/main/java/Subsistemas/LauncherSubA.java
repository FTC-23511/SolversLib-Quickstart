package Subsistemas;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;

@Configurable
public class LauncherSubA extends SubsystemBase {

    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 15.0;
    private static final double LIMELIGHT_LENS_HEIGHT_CM = 28.0;
    private static final double APRILTAG_HEIGHT_CM = 75.6;

    // PID + Feedforward
    private static final double KP = 0.0085;
    private static final double KV = 0.000455;

    private final PController flywheelController = new PController(KP);

    // Hardware
    private final MotorGroup flywheelMotors;
    private final MotorEx motor1;
    private final MotorEx motor2;
    private final ServoEx servoHood;

    private final Limelight3A limelight;

    private static final PwmControl.PwmRange HOOD_RANGE = new PwmControl.PwmRange(500, 2500);
    private static final double HOOD_MIN_ANGLE = 0;
    private static final double HOOD_MAX_ANGLE = 1;

    // Estados
    private boolean shooterRunning = false;
    private double motorPower = 0.0;
    private double motorTicksPerSec = 0.0;
    private double goalDistanceCm = 0.0;
    private double hoodAngleDeg = HOOD_MIN_ANGLE;

    private final InterpLUT angleLut = new InterpLUT();
    private final LUT velocityLut = new LUT();

    public static double TARGET_TICKS_PER_SEC = 1300;
    public static int TARGET_TAG = 20;
    private double HOOD_ANGLE;

    public LauncherSubA(HardwareMap hm, String shooterMotor1, String shooterMotor2, String servoHoodName) {
        motor1 = new MotorEx(hm, shooterMotor1).setCachingTolerance(0.001);
        motor2 = new MotorEx(hm, shooterMotor2).setCachingTolerance(0.001);

        flywheelMotors = new MotorGroup(motor1.setInverted(true), motor2);
        flywheelMotors.setRunMode(Motor.RunMode.RawPower);
        flywheelMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        servoHood = new ServoEx(hm, servoHoodName).setPwm(HOOD_RANGE);
        servoHood.setInverted(true);



        limelight = hm.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // 📌 Inicialización de la Tabla LUT para el ángulo del Hood (cm -> grados)
        angleLut.add(81, 0.35);
        angleLut.add(90.0, 33.0);
        angleLut.add(100.0, 34.0);
        angleLut.add(110.0, 35.0);
        angleLut.createLUT();
    }

    @Override
    public void periodic() {
        // 1. Actualizar datos de visión
        updateGoalDistanceFromVision();

        // 2. Actualizar el ángulo del Hood según la distancia calculada

        servoHood.set(HOOD_ANGLE);


        // 3. Control del Shooter
        motorTicksPerSec = motor2.getCorrectedVelocity();

        if (!shooterRunning) {
            flywheelMotors.set(0.31);

            return;
        }

        flywheelController.setSetPoint(TARGET_TICKS_PER_SEC);
        double pidOutput = flywheelController.calculate(motorTicksPerSec);
        double feedForward = KV * TARGET_TICKS_PER_SEC;

        motorPower = Math.max(-1.0, Math.min(1.0, pidOutput + feedForward));
        flywheelMotors.set(motorPower);
        double velocityError = motorTicksPerSec - TARGET_TICKS_PER_SEC;
        velocityError = Math.abs(velocityError);

    }


    public void updateGoalDistanceFromVision() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            return;
        }

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == TARGET_TAG) {
                double ty = fiducial.getTargetYDegrees();
                double angleToGoal = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
                double heightDiff = APRILTAG_HEIGHT_CM - LIMELIGHT_LENS_HEIGHT_CM;
                goalDistanceCm = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToGoal)));

                break;
            }
        }
    }

    public void setHOOD_ANGLE(double angle) {

        HOOD_ANGLE = angle;
    }

    private boolean hasValidTarget(LLResult result) {
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            return false;
        }

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == TARGET_TAG) {
                return true;
            }
        }
        return false;
    }

    public void setGoalDistance(double distance) {
        this.goalDistanceCm = distance;
    }
    public void setTargetTag(int tag) {
        TARGET_TAG = tag;
    }

    public void startShooter() {
        shooterRunning = true;
        flywheelController.reset();
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
    public double getTicksPerSec() {
        return motorTicksPerSec;
    }

    public double getTicksPerSecError() {
        return TARGET_TICKS_PER_SEC - motorTicksPerSec;
    }

    public double getDistance() {
        return goalDistanceCm;
    }

    public boolean isShooterRunning() {
        return shooterRunning;
    }
}