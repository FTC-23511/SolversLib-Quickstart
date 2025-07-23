package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp()
public class tele_tuning extends LinearOpMode {

    // Outtake servos
    private Servo l_arm_l, l_arm_r;

    @Override
    public void runOpMode() throws InterruptedException {
        l_arm_l = hardwareMap.get(Servo.class, "servo_arm_l");
        l_arm_r = hardwareMap.get(Servo.class, "servo_arm_r");
        l_arm_l.setDirection(Servo.Direction.REVERSE);
        l_arm_l.setPosition(0.5);
        l_arm_r.setPosition(0.5);

        if (opModeInInit()) {
            ;
        }

        // WAIT UNTIL START
        waitForStart();

        // STOP WHEN STOP
        if (isStopRequested()) {
            return;
        }
        // Separate method
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                l_arm_r.setPosition(l_arm_r.getPosition()-0.1);
                l_arm_l.setPosition(l_arm_l.getPosition()-0.1);
            }
            if (gamepad1.bWasPressed()) {
                l_arm_r.setPosition(l_arm_r.getPosition()+0.1);
                l_arm_l.setPosition(l_arm_l.getPosition()+0.1);
            }
        }
    }
}