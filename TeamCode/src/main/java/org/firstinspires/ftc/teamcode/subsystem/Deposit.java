package org.firstinspires.ftc.teamcode.subsystem;

<<<<<<< Updated upstream
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class Deposit {
    public Servo leftArm;
    public Servo rightArm;

    public Servo rightClaw;
    public Servo leftClaw;

    public Servo wrist;

    private final double leftClawOpenPos = 0.07;
    private final double leftClawClosePos = 0.18;

    private final double rightClawOpenPos = 0.59;
    private final double rightClawClosePos = 0.74;

    private final double[] wristPositions = {1, 0.82, 0.64, 0.46, 0.28, 0.08};
    private int wristSplice = 0;

    public Deposit(HardwareMap hardwareMap) {
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        wrist = hardwareMap.get(Servo.class, "wrist");

        rightArm.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(leftClawOpenPos);
        rightClaw.setPosition(rightClawOpenPos);
    }

    public void moveWristLeft() {
        if (wristSplice != 0) {
            wristSplice -= 1;
        }
        wrist.setPosition(wristPositions[wristSplice]);
    }

    public void moveWristRight() {
        if (wristSplice != (wristPositions.length - 1)) {
            wristSplice += 1;
        }
        wrist.setPosition(wristPositions[wristSplice]);
    }

    public void toggleLeftClaw() {
        if (round(wrist.getPosition(), 2) == 0.64 || (round(wrist.getPosition(), 2) == 0.46 || (round(wrist.getPosition(), 2) == 0.28))) {
            if (round(rightClaw.getPosition(), 2) == rightClawClosePos) {
                rightClaw.setPosition(rightClawOpenPos);
            } else {
                rightClaw.setPosition(rightClawClosePos);
            }
        } else {
            if (round(leftClaw.getPosition(), 2) == leftClawClosePos) {
                leftClaw.setPosition(leftClawOpenPos);
            } else {
                leftClaw.setPosition(leftClawClosePos);
            }
        }
    }

    public void toggleRightClaw() {
        if (round(wrist.getPosition(), 2) == 0.64 || round(wrist.getPosition(), 2) == 0.46 || round(wrist.getPosition(), 2) == 0.28) {
            if (round(leftClaw.getPosition(), 2) == leftClawClosePos) {
                leftClaw.setPosition(leftClawOpenPos);
            } else {
                leftClaw.setPosition(leftClawClosePos);
            }
        } else {
            if (round(rightClaw.getPosition(), 2) == rightClawClosePos) {
                rightClaw.setPosition(rightClawOpenPos);
            } else {
                rightClaw.setPosition(rightClawClosePos);
            }
        }
    }
=======
import static org.firstinspires.ftc.teamcode.hardware.Constants.ARM_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ARM_TRANSFER_POS;
import static org.firstinspires.ftc.teamcode.hardware.Constants.LEFT_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.hardware.Constants.LEFT_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.hardware.Constants.MAX_SLIDES_EXTENSION;
import static org.firstinspires.ftc.teamcode.hardware.Constants.RIGHT_CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.hardware.Constants.RIGHT_CLAW_OPEN_POS;
import static org.firstinspires.ftc.teamcode.hardware.Constants.WRIST_BACKDROP_POSITIONS;
import static org.firstinspires.ftc.teamcode.hardware.Constants.WRIST_TRANSFER_POS;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
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
>>>>>>> Stashed changes
}
