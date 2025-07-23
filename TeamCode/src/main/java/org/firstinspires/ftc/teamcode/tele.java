package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp()
public class tele extends LinearOpMode {

    public monkey sys = new monkey();

    // Hardware declarations

    // Drive Motors
    private DcMotorEx motor_fl, motor_bl, motor_fr, motor_br;

    // Lift and Extendo Motors
    private DcMotorEx extendo_motor, lift_l, lift_r;

    // Intake servos/crservos
    private CRServo intake_crservo;
    private Servo pitch_servo, intake_arm;

    // Outtake servos
    private Servo claw_servo, wrist_servo, l_arm_l, l_arm_r;

    // Stilt servos
    private Servo stilt_l, stilt_r;

    // PTO servos
    private Servo pto_l, pto_r;

    @Override()
    public void runOpMode() throws InterruptedException {
        if (opModeInInit()) {
            this.initialize();
        }

        // WAIT UNTIL START
        waitForStart();

        // STOP WHEN STOP
        if (isStopRequested()) {
            return;
        }
        // Separate method
        // I LOVE SEPARATE METHODS(Max hates it but who gaf):)
        while (opModeIsActive()) {
            this.update();
            telemetry.addData("Motor fl", motor_fl.getPower());
            telemetry.addData("Motor fr", motor_fr.getPower());
            telemetry.addData("Motor bl", motor_bl.getPower());
            telemetry.addData("Motor br", motor_br.getPower());
        }
    }

    // Method for initialization
    public void initialize() {

        // Hardware Connections

        // Drive Motor Connection
        motor_fl = hardwareMap.get(DcMotorEx.class, "motor_f_l");
        motor_bl = hardwareMap.get(DcMotorEx.class, "motor_b_l");
        motor_fr = hardwareMap.get(DcMotorEx.class, "motor_f_r");
        motor_br = hardwareMap.get(DcMotorEx.class, "motor_b_r");

        // Lift and Extendo Motor Connection
        extendo_motor = hardwareMap.get(DcMotorEx.class, "motor_extendo");
        lift_l = hardwareMap.get(DcMotorEx.class, "motor_lift_l");
        lift_r = hardwareMap.get(DcMotorEx.class, "motor_lift_r");

        // Intake servo/crservo Connection
        intake_crservo = hardwareMap.get(CRServo.class, "crservo_intake");
        pitch_servo = hardwareMap.get(Servo.class, "servo_pitch");
        intake_arm = hardwareMap.get(Servo.class, "servo_intake_arm");

        pitch_servo.setPosition(0.39);
        intake_arm.setPosition(0.8);

        // Outtake servo Connection
        claw_servo = hardwareMap.get(Servo.class, "servo_claw");
        wrist_servo = hardwareMap.get(Servo.class, "servo_wrist");
        l_arm_l = hardwareMap.get(Servo.class, "servo_arm_l");
        l_arm_r = hardwareMap.get(Servo.class, "servo_arm_r");

        claw_servo.setDirection(Servo.Direction.REVERSE);
        claw_servo.setPosition(0.32);
        wrist_servo.setPosition(0.33);
        l_arm_l.setPosition(0.5);
        l_arm_r.setPosition(0.5);
        l_arm_l.setDirection(Servo.Direction.REVERSE);


        // Stilt servo Connection
        stilt_l = hardwareMap.get(Servo.class, "servo_stilt_l");
        stilt_r = hardwareMap.get(Servo.class, "servo_stilt_r");

        stilt_r.setDirection(Servo.Direction.REVERSE);
        stilt_l.setPosition(0.55);
        stilt_r.setPosition(0.55);

        // PTO servo Connection
        pto_l = hardwareMap.get(Servo.class, "servo_pto_l");
        pto_r = hardwareMap.get(Servo.class, "servo_pto_r");

        pto_l.setPosition(0.28);
        pto_r.setPosition(0.75);
    }


    // Method for recurrency
    public void update() {
        if (gamepad2.leftBumperWasPressed()) {
            sys.i_arm(intake_arm);
        }
        if (gamepad2.rightBumperWasPressed()) {
            sys.pitch(pitch_servo);
        }
        if (gamepad2.dpadRightWasPressed()) {
            sys.wrist(wrist_servo);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            sys.claw(claw_servo);
        }
        // Lift arm positions
        if (gamepad2.xWasPressed()) {
            sys.lift_arm(l_arm_l, l_arm_r, 0);
        }
        if (gamepad2.yWasPressed()) {
            sys.lift_arm(l_arm_l, l_arm_r, 1);
        }
        if (gamepad2.bWasPressed()) {
            sys.lift_arm(l_arm_l, l_arm_r, 2);
        }
        if (gamepad2.aWasPressed()) {
            sys.lift_arm(l_arm_l, l_arm_r, 3);
        }
        if (gamepad2.dpadDownWasPressed()) {
            sys.stilts(stilt_l, stilt_r);
        }
        if (gamepad2.dpadUpWasPressed()) {
            sys.pto(pto_l, pto_r);
        }
        if (gamepad1.dpadUpWasPressed()) {
            sys.transfer(wrist_servo, l_arm_l, l_arm_r, claw_servo, intake_crservo);
        }
        sys.drive(gamepad1, motor_fl, motor_bl, motor_fr, motor_br);
        sys.lift(gamepad2, lift_l, lift_r);
        sys.extendo(gamepad2, extendo_motor);
        sys.intake(gamepad2, intake_crservo);
    }
}
