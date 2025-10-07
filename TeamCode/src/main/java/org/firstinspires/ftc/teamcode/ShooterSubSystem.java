package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class ShooterSubSystem extends SubsystemBase {

    public enum shooterMotorState {
        STOPPED, SHOOTING
    }

    private DcMotor shooterMotor;

    private double kP = 0.000;
    private double kI = 0.000;
    private double kD = 0.000;
    private double kF = 0.000;

    PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public ShooterSubSystem(final HardwareMap hMap) {
        shooterMotor = hMap.get(DcMotor.class, "shooterMotor"); //replace with name of servo when you get it
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setShooterState(shooterMotorState state) {
        switch (state) {
            case STOPPED:
                //shooterMotor.setPower(0.0); //replace with constants
                setPIDTarget(0); //replace with constants
                break;
            case SHOOTING:
                //shooterMotor.setPower(1.0); //replace with constants
                setPIDTarget(0); //replace with constants
                break;
        }
    }

    public void setPIDTarget(int num) {
        pidf.setSetPoint(num);
    }

    public void periodic() {
        shooterMotor.setPower(pidf.calculate(shooterMotor.getCurrentPosition()));
    }
}
