package Subsistemas;


import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

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


    // ================== PID/FF ===================;
    private final PIDFController flyWheelPDIFController = new PIDFController(kP, kI, kD,0.0);

    private SimpleMotorFeedforward flyWheelFController = new SimpleMotorFeedforward(Ks, Kv);

    // ================= HARDWARE =================

    // Shooter
    private final MotorGroup flywheelMotors;
    private final MotorEx motor1;
    private final MotorEx motor2;
    //servo
    private ServoEx servoHood;
;
    private final PwmControl.PwmRange HoodRange = new PwmControl.PwmRange(500,2500);

    private static final double HOOD_MIN_ANGLE = 22.5;
    private static final double HOOD_MAX_ANGLE = 62.74;
    private double flywheelPower;

    // ================= ESTADOS =================

    private boolean shooterRunning = false;

    private double lastKs = Ks;
    private double lastKv = Kv;
    private double motorRPM;
    private double shooterRPM;
    private double goalDistance;
    private double hoodAngle;

    // ================= CONFIGURABLES =================

    //look up tables
    private final InterpLUT hoodLUT = new InterpLUT();
    //private final InterpLUT RPMLUT = new InterpLUT();

    public static double targetRPM = 3000;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double Ks = 0.0;
    public static double Kv = 0.0;


    public LauncherSub(HardwareMap Hm, String shooterMotor1, String shooterMotor2, String ServoHood) {
        // flywheel constructor
        motor1 = new MotorEx(Hm, shooterMotor1).setCachingTolerance(0.01);
        motor2 = new MotorEx(Hm, shooterMotor2).setCachingTolerance(0.01);

        flywheelMotors = new MotorGroup(
                motor1.setInverted(TRUE),
                motor2
                );
        // servo constructor
        servoHood = new ServoEx(Hm,ServoHood,22.5,62.74).setPwm(HoodRange);

        //servo configs
        servoHood.setInverted(FALSE);

        //-------Velocity Control Declaration------

        flywheelMotors.setRunMode(Motor.RunMode.RawPower);
        flywheelMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flyWheelPDIFController.setTolerance(20);

        //---------- Shooter LUTs -----------

        // hood angle LUT
        hoodLUT.add(30.0,22.5);
        hoodLUT.add(60.0,30.0);
        hoodLUT.add(70.0,31.0);
        hoodLUT.add(80.0,32.0);
        hoodLUT.add(90.0,33.0);
        hoodLUT.add(100.0,34.0);
        hoodLUT.add(110.0,35.0);
        hoodLUT.createLUT();

        // flywheel RPM LUT
        //for future use when the shooter is tuned
        /*RPMLUT.add(30,3000);
        RPMLUT.add(60,3100);
        RPMLUT.add(100,3500);
        RPMLUT.add(150,3800);
        RPMLUT.createLUT();*/

    }



    @Override
    public void periodic() {
        //hood ajust
        hoodAngle = hoodLUT.get(goalDistance);
        setHoodAngle(hoodAngle);
        //double CalculatedRPM = RPMLUT.get(goalDistance);


        // Calculate RPM
        motorRPM = motor2.getCorrectedVelocity() * 60.0/28.0;
        shooterRPM = motorRPM/1.5;


        if (!shooterRunning) {
            flywheelMotors.stopMotor();
            flywheelPower = 0.0;
            return;
        }


        //uptate PIDF and FF controllers
        flyWheelPDIFController.setPIDF(kP, kI, kD,0);

        if (lastKs != Ks || lastKv != Kv) {
            flyWheelFController = new SimpleMotorFeedforward(Ks, Kv);
            lastKs = Ks;
            lastKv = Kv;
        }

        // calculate velocity correction
        flyWheelPDIFController.setSetPoint(targetRPM);

        flywheelPower = flyWheelFController.calculate(targetRPM);
        flywheelPower += flyWheelPDIFController.calculate(shooterRPM);

        flywheelMotors.set(clamp(flywheelPower, -1.0, 1.0));

    }
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
    public void setGoalDistance(double distance){
        goalDistance = distance;
    }
    public double getGoalDistance(){
        return goalDistance;
    }
    public void setHoodAngle(double angle) {
        servoHood.set(
                clamp(angle, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE)
        );
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
        if (shooterRunning)
            stopShooter();
        else
            startShooter();
    }

    // ================= TELEMETRY GETTERS =================

    public double getMotorRPM() {
        return motorRPM;
    }

    public double getShooterRPM() {
        return shooterRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPMError() {
        return targetRPM - shooterRPM;
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
