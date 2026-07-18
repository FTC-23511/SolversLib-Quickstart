package org.firstinspires.ftc.teamcode.FlywheelAdvanced;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import javax.net.ssl.CertPathTrustManagerParameters;

public class Flywheel {
    private DcMotorEx m1, m2, Transfer;

    private double encoderCPM = 28;
    private double gearRatio = 1;

    private double kV, kS = 0.3, kP;

    public void init(HardwareMap hwMap) {
        m1 = hwMap.get(DcMotorEx.class, "shooter");
        m2 = hwMap.get(DcMotorEx.class, "shooter2");
        Transfer = hwMap.get(DcMotorEx.class, "Transfer");
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Transfer(double Velocity){
        Transfer.setVelocity(Velocity);
    }

    public double VelocityTranfer(){
        return Transfer.getVelocity();
    }
    public void setMotorPower(double power) {
        m1.setPower(power);
        m2.setPower(power);
    }

    public double getTicksPerSec() {
        return m2.getVelocity();
    }
    public double getRPM(){
        return ((getTicksPerSec()/encoderCPM) * 60)/ gearRatio;
    }
}
