package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSubSystem extends SubsystemBase {

    private DcMotor shooterMotor;

    public static double lastOutput = 0.0;
    public double output = 0.0;

    private double kP = 0.0007;
    private double kI = -0.0001;
    private double kD = 0.000;
    private double kF = 0.000;
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

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    InterpLUT lut = new InterpLUT();

    public ShooterSubSystem(final HardwareMap hMap) {
        shooterMotor = hMap.get(DcMotor.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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
//        shooterPos = shooterMotor.getCurrentPosition();
//        output = pidf.calculate((shooterPos - lastPos) / deltaTime.time(), targetVelocity);
        shooterMotor.setPower(targetVelocity);

//        if (Math.abs(lastOutput - output) > SHOOTER_CACHETHRESHOLD) {
//            shooterMotor.setPower(output);
//            lastOutput = output;
//        }
    }

    public String getShooterOutput() {
        return " " + output;
    }
}
