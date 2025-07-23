package org.firstinspires.ftc.teamcode;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class monkey {
    public void drive(Gamepad gamepad1, DcMotorEx motor_f_l, DcMotorEx motor_b_l, DcMotorEx motor_f_r, DcMotorEx motor_b_r) {
        // IDK WHAT HAPPENED HERE
        double y = -gamepad1.left_stick_y;

        // Multiplier of 1.17 to counteract strafing range
        double x = gamepad1.left_stick_x * 1.17;
        double rx = gamepad1.right_stick_x;

        // Range_limiter containes power within range [-1,1]
        double range_limiter = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double power_fl = -1 * ((y + x + rx) / range_limiter);
        double power_bl = -1 * ((y - x + rx) / range_limiter);
        double power_fr = (y - x - rx) / range_limiter;
        double power_br = (y + x - rx) / range_limiter;

        // Set Power
        motor_f_l.setPower(power_fl);
        motor_b_l.setPower(power_bl);
        motor_f_r.setPower(power_fr);
        motor_b_r.setPower(power_br);
    }
    public void lift(Gamepad gamepad2, DcMotorEx l_l, DcMotorEx l_r) {
        double lift_power = -gamepad2.right_stick_y;

        l_l.setPower(-lift_power);
        l_r.setPower(lift_power);
    }
    public void extendo(Gamepad gamepad2, DcMotorEx extendo_motor) {
        double extendo_power = -gamepad2.left_stick_y;

        extendo_motor.setPower(extendo_power);
    }
    public void intake(Gamepad gamepad2, CRServo cr_intake) {
        double spin_power = gamepad2.right_trigger - gamepad2.left_trigger;

        cr_intake.setPower(spin_power);
    }
    public void i_arm(Servo arm_i) {
        double intake_pos = 0.3;
        double transfer_pos = 0.8;
        double curr_pos = arm_i.getPosition();

        if ((curr_pos > intake_pos - 0.05) && (curr_pos < intake_pos + 0.05)) {
            arm_i.setPosition(transfer_pos);
        } else if ((curr_pos > transfer_pos - 0.05) && (curr_pos < transfer_pos + 0.05)) {
            arm_i.setPosition(intake_pos);
        }
    }
    public void pitch(Servo pitch_srvo) {
        double intake_pos = 0.27;
        double reject_pos = 0.39;
        double curr_pos = pitch_srvo.getPosition();

        if ((curr_pos > intake_pos - 0.05) && (curr_pos < intake_pos + 0.05)) {
            pitch_srvo.setPosition(reject_pos);
        } else if ((curr_pos > reject_pos - 0.05) && (curr_pos < reject_pos + 0.05)) {
            pitch_srvo.setPosition(intake_pos);
        }
    }
    public void wrist(Servo wrist_srvo) {
        double intake_pos = 0.33;
        double spec_sample_pos = 0.0;
        double curr_pos = wrist_srvo.getPosition();

        if ((curr_pos > intake_pos - 0.05) && (curr_pos < intake_pos + 0.05)) {
            wrist_srvo.setPosition(spec_sample_pos);
        } else if ((curr_pos > spec_sample_pos - 0.05) && (curr_pos < spec_sample_pos + 0.05)) {
            wrist_srvo.setPosition(intake_pos);
        }
    }
    public void claw(Servo claw_srvo) {
        double open_pos = 0.32;
        double close_pos = 0.0;
        double curr_pos = claw_srvo.getPosition();

        if ((curr_pos > open_pos - 0.05) && (curr_pos < open_pos + 0.05)) {
            claw_srvo.setPosition(close_pos);
        } else if ((curr_pos > close_pos - 0.05) && (curr_pos < close_pos + 0.05)) {
            claw_srvo.setPosition(open_pos);
        }
    }
    public void lift_arm(Servo arm_l, Servo arm_r, int pos) {
        if (pos==1) {
            arm_r.setPosition(arm_r.getPosition()-0.05);
            arm_l.setPosition(arm_l.getPosition()-0.05);
        } else if (pos==0) {
            arm_r.setPosition(arm_r.getPosition()+0.05);
            arm_l.setPosition(arm_l.getPosition()+0.05);
        }
    }
    public void stilts(Servo stilt_l, Servo stilt_r) {
        double stow_pos = 0.55;
        double active_pos = 0.1;
        double curr_pos = stilt_l.getPosition();

        if ((curr_pos > stow_pos - 0.05) && (curr_pos < stow_pos + 0.05)) {
            stilt_l.setPosition(active_pos);
            stilt_r.setPosition(active_pos);
        } else if ((curr_pos > active_pos - 0.05) && (curr_pos < active_pos + 0.05)) {
            stilt_l.setPosition(stow_pos);
            stilt_r.setPosition(stow_pos);
        }
    }
    public void pto(Servo pto_l, Servo pto_r) {
        double off_pos_l = 0.28;
        double off_pos_r = 0.75;
        double engage_pos_l = 0.2;
        double engage_pos_r = 0.83;
        double curr_pos = pto_l.getPosition();

        if ((curr_pos > off_pos_l - 0.05) && (curr_pos < off_pos_l + 0.05)) {
            pto_l.setPosition(engage_pos_l);
            pto_r.setPosition(engage_pos_r);
        } else if ((curr_pos > engage_pos_l - 0.05) && (curr_pos < engage_pos_l + 0.05)) {
            pto_l.setPosition(off_pos_l);
            pto_r.setPosition(off_pos_r);
        }
    }
    public void transfer(Servo wrist_srvo, Servo arm_l, Servo arm_r, Servo claw_srvo, CRServo intake_crsvro) {
        claw_srvo.setPosition(0.32);
        arm_l.setPosition(0.2);
        arm_r.setPosition(0.2);
        intake_crsvro.setPower(1);
        wrist_srvo.setPosition(0.25);
    }
}