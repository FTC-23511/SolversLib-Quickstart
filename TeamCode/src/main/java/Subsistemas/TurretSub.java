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

    /*
     * Ángulo de la torreta cuando el encoder está en 0.
     *
     * En tu robot:
     * - robot empieza mirando a la izquierda
     * - torreta empieza mirando a la izquierda
     * - heading inicial = 0
     *
     * Entonces la torreta inicia en 0 grados relativos al robot.
     */
    public static double turretStartAngle = 0.0;

    /*
     * Tolerancia en ticks.
     *
     * Con:
     * 28 * 15 * 4.8 = 2016 ticks por vuelta
     *
     * 1 grado ≈ 5.6 ticks
     */
    public static double turretToleranceTicks = 5.0;

    // ================= LIMITS =================

    public static double turretMinAngle = -360.0;
    public static double turretMaxAngle = 360.0;

    // ================= FIELD / TARGET CONFIG =================

    /*
     * Sistema del field:
     *
     * +X = derecha
     * -X = izquierda
     * +Y = arriba
     * -Y = abajo
     *
     * Goal azul:
     * esquina inferior izquierda.
     */
    public static double goalX = -163.0;
    public static double goalY = -152.0;

    /*
     * Distancia mínima para calcular ángulo hacia la goal.
     * Evita inestabilidad cuando el robot está demasiado cerca.
     */
    public static double minimumGoalDistance = 15.0;

    // ================= CORRECTION TOGGLES =================

    /*
     * Corrección por heading:
     *
     * Compensa el giro del robot.
     *
     * Si el robot gira a un lado, la torreta gira al lado contrario
     * para mantener su dirección en el field.
     */
    public static boolean useHeadingCorrection = true;

    /*
     * Corrección por posición:
     *
     * Corrige según cuánto cambia el ángulo hacia la goal
     * desde la posición inicial del robot.
     *
     * Esta corrección NO usa heading.
     */
    public static boolean usePositionCorrection = true;

    /*
     * Si la corrección del heading va al revés,
     * cambia esto a -1.0.
     */
    public static double headingCorrectionDirection = -1.0;

    /*
     * Si la corrección por posición va al revés,
     * cambia esto a -1.0.
     */
    public static double positionCorrectionDirection = -1.0;

    /*
     * Offset manual para calibrar puntería.
     *
     * Si siempre queda un poco a la izquierda o derecha,
     * ajusta este valor desde dashboard.
     */
    public static double manualAimOffsetDegrees = 0.0;

    /*
     * Si las dos correcciones están apagadas:
     *
     * true  = mantiene la posición actual
     * false = regresa a turretStartAngle
     */
    public static boolean holdCurrentAngleWhenCorrectionsDisabled = true;

    /*
     * Si true, cuando la posición está apagada, se borra la referencia
     * inicial de la goal para capturar una nueva cuando se vuelva a activar.
     */
    public static boolean resetGoalReferenceWhenPositionDisabled = false;

    // ================= STATE =================

    private boolean enabled = true;

    private double heading = 0.0;

    private double currentTicks = 0.0;
    private double currentAngle = turretStartAngle;

    private double desiredAngle = turretStartAngle;
    private double targetAngle = turretStartAngle;
    private double targetTicks = 0.0;

    private double errorTicks = 0.0;
    private double turretPower = 0.0;

    private double robotX = 0.0;
    private double robotY = 0.0;

    private double goalDistance = 0.0;
    private double goalBearing = 0.0;

    /*
     * Referencia inicial del ángulo hacia la goal.
     *
     * Esto permite separar la corrección por posición
     * de la corrección por heading.
     */
    private double initialGoalBearing = 0.0;
    private boolean goalReferenceCaptured = false;

    /*
     * Correcciones separadas.
     */
    private double headingCorrectionAngle = 0.0;
    private double positionCorrectionAngle = 0.0;

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

        /*
         * OJO:
         * Esto hace que el Pinpoint tome la posición actual como:
         *
         * X = 0
         * Y = 0
         * heading = 0
         *
         * Solo es correcto si el robot empieza en la posición
         * que quieres usar como referencia.
         */
        pinpoint.resetPosAndIMU();
    }

    // ================= PERIODIC =================

    @Override
    public void periodic() {

        // Actualizar Pinpoint
        pinpoint.update();

        // Leer encoder de la torreta
        currentTicks = turretMotor.getCurrentPosition();
        currentAngle = turretTicksToDegrees(currentTicks);

        // Si Pinpoint no está listo, no mover la torreta
        if (pinpoint.getDeviceStatus()
                != GoBildaPinpointDriver.DeviceStatus.READY) {

            stopMotor();
            return;
        }

        // Leer odometría
        heading = pinpoint.getHeading(
                UnnormalizedAngleUnit.DEGREES
        );

        robotX = pinpoint.getPosX(distanceUnit);
        robotY = pinpoint.getPosY(distanceUnit);

        // Calcular vector desde el robot hacia la goal
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        goalDistance = Math.hypot(deltaX, deltaY);

        /*
         * Ángulo absoluto hacia la goal en el field.
         *
         * Sistema:
         *
         * 0°    = derecha
         * 90°   = arriba
         * 180°  = izquierda
         * -90°  = abajo
         */


        goalBearing = Math.toDegrees(
                Math.atan2(deltaY, deltaX)
        );

        goalBearing = normalizeDegrees(goalBearing);

        /*
         * Capturamos la dirección inicial hacia la goal.
         *
         * Esta referencia permite que la corrección de posición
         * sea independiente del heading.
         *
         * positionCorrectionAngle significa:
         * "cuánto cambió el ángulo hacia la goal desde el inicio".
         */
        if (!goalReferenceCaptured) {
            initialGoalBearing = turretStartAngle;
            goalReferenceCaptured = true;
        }

        positionCorrectionAngle = normalizeDegrees(
                goalBearing - initialGoalBearing
        );

        positionCorrectionAngle = normalizeDegrees(
                positionCorrectionAngle * positionCorrectionDirection
        );




        /*
         * Corrección por heading.
         *
         * Como el robot inicia con heading = 0 mirando a la izquierda
         * y la torreta también inicia a la izquierda:
         *
         * headingCorrectionAngle = -heading
         *
         * El signo negativo es porque la torreta debe compensar
         * en sentido contrario al giro del robot.
         */
        headingCorrectionAngle = -heading * headingCorrectionDirection;
        headingCorrectionAngle = normalizeDegrees(headingCorrectionAngle);

        if (!enabled) {
            stopMotor();
            return;
        }

        /*
         * Construcción del ángulo deseado por partes:
         *
         * turretStartAngle
         * + corrección por heading si está activa
         * + corrección por posición si está activa
         * + offset manual
         */
        if (!useHeadingCorrection && !usePositionCorrection) {

            if (holdCurrentAngleWhenCorrectionsDisabled) {
                desiredAngle = currentAngle;
            } else {
                desiredAngle = turretStartAngle;
            }

            if (resetGoalReferenceWhenPositionDisabled) {
                resetGoalReference();
            }

        } else {

            desiredAngle = turretStartAngle;

            if (useHeadingCorrection && usePositionCorrection) {
                desiredAngle += (headingCorrectionAngle + positionCorrectionAngle);
            }

            if (usePositionCorrection && !useHeadingCorrection) {
                desiredAngle += positionCorrectionAngle;
            }
            if  (!usePositionCorrection && useHeadingCorrection) {
                desiredAngle += headingCorrectionAngle;
            }
        }

        desiredAngle += manualAimOffsetDegrees;

        desiredAngle = normalizeDegrees(desiredAngle);

        /*
         * Buscar el ángulo equivalente más cercano
         * dentro de los límites físicos de la torreta.
         */
        targetAngle = findBestTurretTarget(
                desiredAngle,
                currentAngle
        );

        targetTicks = degreesToTurretTicks(targetAngle);

        // Redondear a una décima de tick
        targetTicks = Math.round(targetTicks * 10.0) / 10.0;

        // Actualizar PID por si cambias valores desde dashboard
        turretController.setPIDF(
                turretKP,
                turretKI,
                turretKD,
                0
        );

        turretController.setSetPoint(targetTicks);

        turretPower = turretController.calculate(currentTicks);

        turretPower = clamp(
                turretPower,
                -turretMaxPower,
                turretMaxPower
        );

        errorTicks = targetTicks - currentTicks;

        // Detener dentro de tolerancia
        if (Math.abs(errorTicks) <= turretToleranceTicks) {
            turretPower = 0.0;
        }

        // Límite mínimo de software
        if (currentAngle <= turretMinAngle && turretPower < 0.0) {
            turretPower = 0.0;
        }

        // Límite máximo de software
        if (currentAngle >= turretMaxAngle && turretPower > 0.0) {
            turretPower = 0.0;
        }

        turretMotor.set(turretPower);
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

    private double degreesToTurretTicks(
            double turretAngle
    ) {
        return (turretAngle - turretStartAngle)
                * getTicksPerTurretRev()
                / 360.0;
    }

    private double turretTicksToDegrees(
            double ticks
    ) {
        return turretStartAngle
                + ticks
                * 360.0
                / getTicksPerTurretRev();
    }

    private double findBestTurretTarget(
            double desiredAngle,
            double currentAngle
    ) {
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;

        boolean foundValidTarget = false;

        /*
         * Busca una representación equivalente del ángulo
         * dentro de los límites físicos.
         *
         * Ejemplo:
         * -170° y 190° apuntan a la misma dirección.
         */
        for (int rotation = -8; rotation <= 8; rotation++) {

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

        /*
         * Si no hay un ángulo equivalente dentro de los límites,
         * usa el límite más cercano.
         */
        return clamp(
                desiredAngle,
                turretMinAngle,
                turretMaxAngle
        );
    }

    // ================= MAIN ENABLE / DISABLE =================

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

    // ================= CORRECTION CONTROLS =================

    public void enableHeadingCorrection() {
        useHeadingCorrection = true;
        turretController.reset();
    }

    public void disableHeadingCorrection() {
        useHeadingCorrection = false;
        turretController.reset();
    }

    public void toggleHeadingCorrection() {
        useHeadingCorrection = !useHeadingCorrection;
        turretController.reset();
    }

    public boolean isHeadingCorrectionEnabled() {
        return useHeadingCorrection;
    }

    public void enablePositionCorrection() {
        usePositionCorrection = true;
        turretController.reset();
    }

    public void disablePositionCorrection() {
        usePositionCorrection = false;

        if (resetGoalReferenceWhenPositionDisabled) {
            resetGoalReference();
        }

        turretController.reset();
    }

    public void togglePositionCorrection() {
        usePositionCorrection = !usePositionCorrection;

        if (!usePositionCorrection && resetGoalReferenceWhenPositionDisabled) {
            resetGoalReference();
        }

        turretController.reset();
    }

    public boolean isPositionCorrectionEnabled() {
        return usePositionCorrection;
    }

    public void useOnlyHeadingCorrection() {
        useHeadingCorrection = true;
        usePositionCorrection = false;

        if (resetGoalReferenceWhenPositionDisabled) {
            resetGoalReference();
        }

        turretController.reset();
    }

    public void useOnlyPositionCorrection() {
        useHeadingCorrection = false;
        usePositionCorrection = true;
        turretController.reset();
    }

    public void useFullCorrection() {
        useHeadingCorrection = true;
        usePositionCorrection = true;
        turretController.reset();
    }

    public void disableAllCorrections() {
        useHeadingCorrection = false;
        usePositionCorrection = false;

        if (resetGoalReferenceWhenPositionDisabled) {
            resetGoalReference();
        }

        turretController.reset();
    }

    // ================= OFFSET CONTROLS =================

    public void setManualAimOffsetDegrees(
            double offsetDegrees
    ) {
        manualAimOffsetDegrees = offsetDegrees;
    }

    public void addManualAimOffsetDegrees(
            double offsetDegrees
    ) {
        manualAimOffsetDegrees += offsetDegrees;
    }

    public void resetManualAimOffset() {
        manualAimOffsetDegrees = 0.0;
    }

    // ================= RESET METHODS =================

    public void resetHeading() {
        stopMotor();

        /*
         * Solo llama esto si el robot está físicamente
         * mirando hacia la izquierda.
         */
        pinpoint.setHeading(
                0.0,
                AngleUnit.DEGREES
        );

        pinpoint.update();

        heading = 0.0;
        headingCorrectionAngle = 0.0;

        desiredAngle = turretStartAngle;
        targetAngle = turretStartAngle;
        targetTicks = 0.0;
        errorTicks = 0.0;

        turretController.reset();
    }

    public void resetEncoder() {
        stopMotor();

        /*
         * Solo llama esto si la torreta está físicamente
         * apuntando hacia la izquierda.
         */
        turretMotor.stopAndResetEncoder();

        turretMotor.setRunMode(
                Motor.RunMode.RawPower
        );

        currentTicks = 0.0;
        currentAngle = turretStartAngle;

        targetTicks = 0.0;
        targetAngle = turretStartAngle;
        errorTicks = 0.0;

        turretController.reset();
    }

    public void resetGoalReference() {
        goalReferenceCaptured = false;
        initialGoalBearing = 0.0;
        goalBearing = 0.0;
        positionCorrectionAngle = 0.0;

        turretController.reset();
    }

    public void resetAll() {
        stopMotor();

        resetEncoder();
        resetHeading();
        resetGoalReference();

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

    public double getInitialGoalBearing() {
        return initialGoalBearing;
    }

    public boolean isGoalReferenceCaptured() {
        return goalReferenceCaptured;
    }

    public double getHeadingCorrectionAngle() {
        return headingCorrectionAngle;
    }

    public double getPositionCorrectionAngle() {
        return positionCorrectionAngle;
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