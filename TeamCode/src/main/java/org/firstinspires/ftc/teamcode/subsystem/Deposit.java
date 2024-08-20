package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Deposit extends SubsystemBase {
    private final RobotHardware robot = RobotHardware.getInstance();
    private static final PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    public int pixelHeight = 1;
    public int wristIndex = 4;
    private double target;

    // Between transfer and backdrop position
    public boolean armTransfer = true;

    // Between retracted and extended
    public boolean slidesRetracted;

    // Between open and closed
    public boolean leftClawOpen;
    public boolean rightClawOpen;

    // Default will reset deposit to transfer position (unpowered claw servos depending on auto vs tele-op)
    public void init() {
        slidePIDF.setTolerance(10, 10);
        setArmTransfer(true);
        setWristTransfer();
        setSlideTarget(0);
    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_SLIDES_EXTENSION), 0);
        slidePIDF.setSetPoint(target);
    }

    public void autoSetSlidePower() {
        double power = slidePIDF.calculate(robot.liftEncoder.getPosition(), target);
        robot.liftRight.setPower(power);
        robot.liftLeft.setPower(-power);

        // Slides are only retracted once stopped and at a target of 0
        slidesRetracted = ((target <= 0)) && (this.reached());
    }

    // Returns if slides have reached the target
    public boolean reached() {
        return (slidePIDF.atSetPoint());
    }

    public void setArmTransfer(boolean armTransfer) {
        if (armTransfer != this.armTransfer) {
            robot.rightArm.setPosition(armTransfer ? ARM_TRANSFER_POS : ARM_BACKDROP_POS);
            robot.leftArm.setPosition(armTransfer ? -ARM_TRANSFER_POS + 1 : -ARM_BACKDROP_POS + 1);
            this.armTransfer = armTransfer;
        }
    }

    // Be careful with these 2 methods to make sure armState is at the relevant state/position
    public void updateWrist(int position) {
        if (position != wristIndex) {
            wristIndex = position;
            robot.wrist.setPosition(WRIST_BACKDROP_POSITIONS[wristIndex]);
        }
    }

    public void setWristTransfer() {
        if (robot.wrist.getPosition() != round(WRIST_TRANSFER_POS, 2)) {
            robot.wrist.setPosition(WRIST_TRANSFER_POS);
        }
    }

    public void setClaw(boolean leftClawOpen, boolean rightClawOpen) {
        if (leftClawOpen != this.leftClawOpen) {
            if (leftClawOpen) {
                robot.leftClaw.setPosition(LEFT_CLAW_OPEN_POS);
            } else {
                robot.leftClaw.setPosition(LEFT_CLAW_CLOSE_POS);
            }

            this.leftClawOpen = leftClawOpen;
        }

        if (rightClawOpen != this.rightClawOpen) {
            if (leftClawOpen) {
                robot.rightClaw.setPosition(RIGHT_CLAW_OPEN_POS);
            } else {
                robot.rightClaw.setPosition(RIGHT_CLAW_CLOSE_POS);
            }

            this.rightClawOpen = rightClawOpen;
        }
    }

    @Override
    public void periodic() {
        autoSetSlidePower();
    }
}
