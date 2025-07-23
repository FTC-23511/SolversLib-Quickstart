package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp()
public class test extends LinearOpMode {

    public ServoImplEx test_servo, intake_arm, wrist_servo, pitch_servo, claw_servo, test_servo_l;

    @Override()
    public void runOpMode() {
        if (opModeInInit()) {
            wrist_servo = hardwareMap.get(ServoImplEx.class, "servo_wrist");
            wrist_servo.setPosition(0.33);
            pitch_servo = hardwareMap.get(ServoImplEx.class, "servo_pitch");
            pitch_servo.setPosition(0.39);
            intake_arm = hardwareMap.get(ServoImplEx.class, "servo_intake_arm");
            intake_arm.setPosition(0.75);
            claw_servo = hardwareMap.get(ServoImplEx.class, "servo_claw");
            claw_servo.setPosition(0.0);
            //
            test_servo_l = hardwareMap.get(ServoImplEx.class, "servo_arm_l");
            test_servo_l.setPosition(0.92);
            test_servo = hardwareMap.get(ServoImplEx.class, "servo_arm_l");
            test_servo.setDirection(Servo.Direction.REVERSE);
            test_servo.setPosition(0.92);
        }

        waitForStart();

        while (opModeIsActive()) {
        }
    }

}