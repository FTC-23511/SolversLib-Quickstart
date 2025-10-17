package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSubSystem extends SubsystemBase {

    public static double lastOutput = 0.0;
    public double output = 0.0;

    private double kP = 0.00002;
    private double kI = 0.000;
    private double kD = 0.0002;
    private double kS = 500.0;
    private double kV = 0.0005;
    private double kA = 0.000;
    Motor shooterMotor;
    public int getShooterPosition() {
        return shooterPos;
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }
    int shooterPos = 0;
    public double targetVelocity = 0;
    ElapsedTime deltaTime = new ElapsedTime();
    int lastPos = 0;

    InterpLUT lut = new InterpLUT();

    public ShooterSubSystem(final HardwareMap hMap) {
        shooterMotor = new Motor(hMap, "shooter", Motor.GoBILDA.RPM_312);

        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        shooterMotor.setVeloCoefficients(kP, kI, kD);
        shooterMotor.setFeedforwardCoefficients(kS, kV, kA);
        shooterMotor.set(0.0);


        shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        //lut.add(1.0,1.0);
        //lut.createLUT();
    }

    public void setTargetFromDistance(double distance) {
        targetVelocity = (lut.get(distance));
    }

    public void setTargetVelocity(double num) {
        targetVelocity = num;
    }

    public void periodic() {
        shooterMotor.set(targetVelocity);
    }

    public String getShooterOutput() {
        return " " + output;
    }
}
