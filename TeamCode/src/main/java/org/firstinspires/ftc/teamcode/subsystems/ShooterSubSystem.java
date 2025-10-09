package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterSubSystem extends SubsystemBase {

    public enum shooterMotorState {
        STOPPED, SHOOTING
    }

    private DcMotor shooterMotor;

    public static double lastOutput = 0.0;
    public static double output = 0.0;

    private double kP = 0.000;
    private double kI = 0.005;
    private double kD = 0.000;
    private double kF = 0.000;
    ElapsedTime deltaTime = new ElapsedTime();
    int lastPos = 0;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);
    InterpLUT lut = new InterpLUT();

    public ShooterSubSystem(final HardwareMap hMap) {
        shooterMotor = hMap.get(DcMotor.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lut.add(1.0,1.0);
        //lut.createLUT();
    }

    public void setTargetFromDistance(double distance) {
        setTargetFromDistance(lut.get(distance));
    }

    public void setTargetVelocity(double num) {
        pidf.setSetPoint(num);
    }

    public void periodic() {
        lastOutput = output;
        int shooterPos = shooterMotor.getCurrentPosition();
        output = pidf.calculate((lastPos - shooterPos) / deltaTime.time());

        if (Math.abs(lastOutput - output) < SHOOTER_CACHETHRESHOLD) {
            shooterMotor.setPower(output);
        }
    }
}
