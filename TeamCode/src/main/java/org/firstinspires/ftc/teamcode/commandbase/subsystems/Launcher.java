package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public static PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);

    public void init() {
        setRamp(false);
        setHood(MIN_HOOD_ANGLE);
        setFlywheel(0);
    }

    public void setFlywheel(double vel) {
        flywheelController.setSetPoint(vel * M_S_TO_TICKS);
    }

    public double getFlywheelTarget() {
        return flywheelController.getSetPoint();
    }

    private void updateFlywheel() {
        if (getFlywheelTarget() == 0) { // Don't unnecessarily decelerate to a stop (just causes current spike)
            robot.launchMotors.set(0);
        } else {
            flywheelController.setF(FLYWHEEL_PIDF_COEFFICIENTS.f / (robot.getVoltage() / 12));
            robot.launchMotors.set(
                    flywheelController.calculate(robot.launchEncoder.getCorrectedVelocity())
            );
        }
    }

    public void setRamp(boolean engaged) {
        robot.rampServo.set(engaged ? RAMP_ENGAGED : RAMP_DISENGAGED);
    }

    public void setHood(double angle) {
        // Solved from proportion (targetServo - minServo) / servoRange = (targetAngle - minAngle) / angleRange
        robot.hoodServo.set(
                (angle - MIN_HOOD_ANGLE) / (MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * (MAX_HOOD_SERVO_POS - MIN_HOOD_SERVO_POS) + MIN_HOOD_SERVO_POS
        );
    }

    public boolean readyToLaunch() {
        return flywheelController.atSetPoint();
    }

    @Override
    public void periodic() {
        updateFlywheel();
    }
}
