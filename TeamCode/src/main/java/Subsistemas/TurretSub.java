package Subsistemas;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@Configurable
public class TurretSub extends SubsystemBase {

    // ================= HARDWARE =================

    private final MotorEx turretMotor;
    private final GoBildaPinpointDriver pinpoint;

    private final PIDFController turretController;

    // ================= PID =================

    public static double turretKP = 0.015;
    public static double turretKI = 0.0;
    public static double turretKD = 0.0;

    // ================= ENCODER / GEAR RATIO =================

    public static double motorEncoderTicksPerRev = 28.0;
    public static double motorGearboxRatio = 10.43290;
    public static double turretExternalGearRatio = 4.8;

    // ================= POWER =================

    public static double turretMaxPower = 0.50;




    public static double turretToleranceTicks = 2.0;

    // ================= LIMITS =================

    public static double turretMinAngle = -270.0;
    public static double turretMaxAngle = 270.0;

    // ================= FIELD / TARGET CONFIG =================


    public static double goalX = -152.4;
    public static double goalY = -152.4;



    private double xOffset = 0;
    private double yOffset = 0;
    private double headingOffset = 0;


    // ================= CORRECTION TOGGLES =================




    public static boolean usePositionCorrection = true;




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

    private double robotX = 0.0;
    private double robotY = 0.0;

    private double goalDistance = 0.0;
    private double goalBearing = 0.0;

    public static double initHeading = 0;
    public static double initX = 0;
    public static double initY = 0;









    private double headingCorrectionAngle = 0.0;


    private final DistanceUnit distanceUnit = DistanceUnit.CM;

    // ================= CONSTRUCTOR =================

    public TurretSub(
            HardwareMap hardwareMap,
            String turretMotorName,
            String pinpointName
    ) {
        turretMotor = new MotorEx(hardwareMap, turretMotorName);

        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        turretMotor.stopAndResetEncoder();

        turretMotor.setRunMode(Motor.RunMode.RawPower);

        turretController = new PIDFController(
                turretKP,
                turretKI,
                turretKD,
                0
        );

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                pinpointName
        );

        pinpoint.setOffsets(
                27.05,
                199.02,
                DistanceUnit.MM
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

        pinpoint.update();


        setFieldPose(initX,initY,initHeading);


    }

    // ================= PERIODIC =================

    @Override
    public void periodic() {
        // ================= UPDATE SENSORS =================

        pinpoint.update();

        currentTicks = turretMotor.getCurrentPosition();

        currentAngle =
                ticksToDegrees(currentTicks);


        heading = normalizeDegrees(
                pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES)
                        + headingOffset
        );

        robotX =
                pinpoint.getPosX(distanceUnit)
                        + xOffset;

        robotY =
                pinpoint.getPosY(distanceUnit)
                        + yOffset;

        // ================= CALCULATE FIELD TARGET =================

        if (usePositionCorrection) {

            double deltaX = goalX - robotX;
            double deltaY = goalY - robotY;

            goalDistance = Math.hypot(deltaX, deltaY);

            desiredAngle = Math.toDegrees(
                    Math.atan2(deltaX, -deltaY)
            );

        } else {

            goalDistance = 0.0;

            desiredAngle = normalizeDegrees(
                    heading - currentAngle
            );

        }

        // ================= RELATIVE TARGET =================

        double relativeTargetAngle =
                normalizeDegrees(
                        desiredAngle
                                - heading
                                + manualAimOffsetDegrees
                );

        // ================= BEST ROTATION =================

        targetAngle =
                findBestTurretTarget(
                        -relativeTargetAngle,
                        currentAngle
                );

        // ================= TICKS =================

        targetTicks =
                degreesToTicks(
                        targetAngle
                );

        if (!enabled) {
            stopMotor();
            return;
        }

        turretController.setPIDF(
                turretKP,
                turretKI,
                turretKD,
                0
        );

        turretController.setSetPoint(
                targetTicks
        );

        turretPower =
                turretController.calculate(
                        currentTicks
                );

        turretPower = clamp(
                turretPower,
                -turretMaxPower,
                turretMaxPower
        );

        errorTicks =
                targetTicks
                        - currentTicks;

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

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }

    public double getTicksPerTurretRev() {
        return motorEncoderTicksPerRev
                * motorGearboxRatio
                * turretExternalGearRatio;
    }


    private double degreesToTicks(double degrees){
        return degrees * getTicksPerTurretRev() / 360.0;
    }

    private double ticksToDegrees(double ticks){
        return ticks * 360.0 / getTicksPerTurretRev();
    }
    public void setFieldPose(
            double x,
            double y,
            double headingDeg
    ) {

        pinpoint.update();

        xOffset =
                x - pinpoint.getPosX(distanceUnit);

        yOffset =
                y - pinpoint.getPosY(distanceUnit);

        headingOffset =
                headingDeg - pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
    }
    private double findBestTurretTarget(
            double desiredAngle,
            double currentAngle
    ) {
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;

        boolean foundValidTarget = false;

        // Reducido el rango de búsqueda ya que la torreta solo tiene un rango de 540 grados (-270 a 270)
        for (int rotation = -2; rotation <= 2; rotation++) {

            double candidate =
                    desiredAngle + 360.0 * rotation;

            boolean insideLimits =
                    candidate >= turretMinAngle
                            && candidate <= turretMaxAngle;

            if (!insideLimits) {
                continue;
            }

            double distance = Math.abs(
                    candidate - currentAngle
            );

            if (distance < bestDistance) {
                bestDistance = distance;
                bestAngle = candidate;
                foundValidTarget = true;
            }
        }

        if (foundValidTarget) {
            return bestAngle;
        }

        return clamp(
                desiredAngle,
                turretMinAngle,
                turretMaxAngle
        );
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


        pinpoint.setHeading(
                0.0,
                AngleUnit.DEGREES
        );

        pinpoint.update();

        heading = 0.0;
        headingCorrectionAngle = 0.0;

        desiredAngle = 0;
        targetAngle = 0;
        targetTicks = 0.0;
        errorTicks = 0.0;

        turretController.reset();
    }

    public void resetEncoder() {
        stopMotor();

        turretMotor.stopAndResetEncoder();

        turretMotor.setRunMode(
                Motor.RunMode.RawPower
        );

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



    // ================= TELEMETRY GETTERS =================

    public double getHeading() {
        return heading;
    }

    public double getCurrentTicks() {
        return currentTicks;
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getTargetTicks() {
        return targetTicks;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getErrorTicks() {
        return errorTicks;
    }

    public double getAppliedPower() {
        return turretPower;
    }

    public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() {
        return pinpoint.getDeviceStatus();
    }

    public double getPinpointFrequency() {
        return pinpoint.getFrequency();
    }

    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    public double getGoalDistance() {
        return goalDistance;
    }

    public double getGoalBearing() {
        return goalBearing;
    }





    public double getHeadingCorrectionAngle() {
        return headingCorrectionAngle;
    }



    public double getManualAimOffsetDegrees() {
        return manualAimOffsetDegrees;
    }

    public double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity(
                UnnormalizedAngleUnit.DEGREES
        );
    }
}