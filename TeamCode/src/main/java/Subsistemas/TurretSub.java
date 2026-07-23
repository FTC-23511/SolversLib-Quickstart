package Subsistemas;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class TurretSub extends SubsystemBase {


    // ================= HARDWARE =================
    private final MotorEx turretMotor;
    private final PController turretController;


    // ================= P Controller =================
    public static double turretKP = 0.015;

    // ================= ENCODER / GEAR RATIO =================
    public static double motorEncoderTicksPerRev = 28.0;
    public static double motorGearboxRatio = 10.43290;
    public static double turretExternalGearRatio = 4.8;

    // ================= POWER =================
    public static double turretMaxPower = 0.50;
    public static double turretToleranceTicks = 2.0;
    // ================= LIMITS =================
    public static double turretMinAngle = -141;
    public static double turretMaxAngle = 155;
    // ================= FIELD / TARGET CONFIG =================
    public static double goalX = 144;
    public static double goalY = 144;
    public static double manualAimOffsetDegrees = 0;
    // ================= STATE =================
    private boolean enabled = true;
    private double heading = 0.0;
    private double currentTicks = 0.0;
    private double currentAngle = 0;
    private double desiredAngle = 0;
    private double targetAngle = 0;
    private double targetTicks = 0.0;
    private double errorTicks = 0.0;
    private double turretPower = 0.0;
    private double goalDistance = 0.0;
    private static Pose robotPose;
    // ================= CONSTRUCTOR =================
    public TurretSub(HardwareMap hardwareMap, String turretMotorName) {
        turretMotor = new MotorEx(hardwareMap, turretMotorName);

        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        turretMotor.stopAndResetEncoder();

        turretMotor.setRunMode(Motor.RunMode.RawPower);



        turretController = new PController(turretKP);
    }
    // ================= PERIODIC =================

    @Override
    public void periodic() {

        // ================= UPDATE SENSORS =================
        currentTicks = turretMotor.getCurrentPosition();

        currentAngle = ticksToDegrees(currentTicks);

        double robotX = robotPose.getX();

        double robotY = robotPose.getY();

        heading = robotPose.getHeading();

        heading = Math.toDegrees(heading);

        // ================= CALCULATE FIELD TARGET =================
            double deltaX = goalX - robotX;
            double deltaY = goalY - robotY;

            desiredAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // ================= RELATIVE TARGET =================

        double relativeTargetAngle = normalizeDegrees(desiredAngle - heading + manualAimOffsetDegrees);

        // ================= BEST ROTATION =================

        targetAngle = findBestTurretTarget(-relativeTargetAngle, currentAngle);

        // ================= TICKS =================

        targetTicks = degreesToTicks(targetAngle);

        if (!enabled) {
            stopMotor();
            return;
        }

        turretController.setP(turretKP);

        turretController.setSetPoint(targetTicks);

        turretPower = turretController.calculate(currentTicks);

        turretPower = clamp(turretPower, -turretMaxPower, turretMaxPower);

        errorTicks = targetTicks - currentTicks;

        if (Math.abs(errorTicks)
                <= turretToleranceTicks) {
            turretPower = 0.0;
        }

        if (currentAngle <= turretMinAngle
                && turretPower < 0) {
            turretPower = 0.0;
        }

        if (currentAngle >= turretMaxAngle
                && turretPower > 0) {
            turretPower = 0.0;
        }

        turretMotor.set(
                turretPower
        );
    }
    // ================= MATH HELPERS =================


    public void setPose(Pose RobotPose){
        robotPose = RobotPose;
    }
    public void setManualAimOffsetDegrees(double offset){

        manualAimOffsetDegrees = offset;
    }
    public void setGoalX(double GoalX){
        goalX = GoalX;
    }
    public void setGoalY(double GoalY){
        goalY = GoalY;
    }
    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(minimum, Math.min(maximum, value));
    }
    public double getTicksPerTurretRev() {return motorEncoderTicksPerRev * motorGearboxRatio * turretExternalGearRatio;}

    private double degreesToTicks(double degrees){
        return degrees * getTicksPerTurretRev() / 360.0;
    }

    private double ticksToDegrees(double ticks){
        return ticks * 360.0 / getTicksPerTurretRev();
    }
    private double findBestTurretTarget(double desiredAngle, double currentAngle) {
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;

        boolean foundValidTarget = false;


        for (int rotation = -1; rotation <= 1; rotation++) {

            double candidate = desiredAngle + 360.0 * rotation;

            boolean insideLimits = candidate >= turretMinAngle && candidate <= turretMaxAngle;

            if (!insideLimits) {
                continue;
            }

            double distance = Math.abs(candidate - currentAngle);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestAngle = candidate;
                foundValidTarget = true;
            }
        }

        if (foundValidTarget) {
            return bestAngle;
        }

        return clamp(desiredAngle, turretMinAngle, turretMaxAngle);
    }

    public void enable() {
        turretController.reset();
        enabled = true;
    }

    public void disable() {
        enabled = false;
        turretController.reset();
        stopMotor();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void toggleEnabled() {
        if (enabled) {
            disable();
        } else {
            enable();
        }
    }


    public void resetHeading() {
        stopMotor();
        heading = 0.0;
        desiredAngle = 0;
        targetAngle = 0;
        targetTicks = 0.0;
        errorTicks = 0.0;
        turretController.reset();
    }

    public void resetEncoder() {
        stopMotor();

        turretMotor.stopAndResetEncoder();
        turretMotor.setRunMode(Motor.RunMode.RawPower);

        currentTicks = 0.0;
        currentAngle = 0;

        targetTicks = 0.0;
        targetAngle = 0;
        errorTicks = 0.0;

        turretController.reset();
    }
    public void resetAll() {
        stopMotor();

        resetEncoder();
        resetHeading();
        manualAimOffsetDegrees = 0.0;
        turretController.reset();
    }
    public void stopMotor() {
        turretMotor.stopMotor();
        turretPower = 0.0;
    }

    public Pose getRobotPose() {return robotPose;}
    // ================= TELEMETRY GETTERS =================
    public double getGoalDistance() {return goalDistance;}

    public double getCurrentAngle() {return currentAngle;}

    public double getTargetAngle() {return targetAngle;}

    public double getAppliedPower() {return turretPower;}
}