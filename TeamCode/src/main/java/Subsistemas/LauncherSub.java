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

@Configurable
public class LauncherSub extends SubsystemBase {

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


    // Estados
    private boolean shooterRunning = false;
    private double motorPower = 0.0;
    private double motorTicksPerSec = 0.0;
    private double goalDistanceCm = 0.0;


    private final InterpLUT angleLut = new InterpLUT();
    private final InterpLUT velocityLut = new InterpLUT();

    private double TARGET_TICKS_PER_SEC;
    public int TARGET_TAG = 24;
    private double HOOD_ANGLE;

    public LauncherSub(HardwareMap hm, String shooterMotor1, String shooterMotor2, String servoHoodName) {
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
        angleLut.add(81, 0.3);
        angleLut.add(90.0, 0.32);
        angleLut.add(104.0, 0.33);
        angleLut.add(240, 0.5);
        angleLut.createLUT();

        velocityLut.add(81, 990);
        velocityLut.add(104, 1010);
        velocityLut.add(160, 1160);
        velocityLut.add(240, 1430);

    }

    @Override
    public void periodic() {
        // 1. Actualizar datos de visión
        updateGoalDistanceFromVision();

        // 2. Actualizar el ángulo del Hood según la distancia calculada
        HOOD_ANGLE = angleLut.get(goalDistanceCm);

        servoHood.set(HOOD_ANGLE);

        TARGET_TICKS_PER_SEC = velocityLut.get(goalDistanceCm);



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
    public void setHOOD_ANGLE(double angle) {

        HOOD_ANGLE = angle;
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