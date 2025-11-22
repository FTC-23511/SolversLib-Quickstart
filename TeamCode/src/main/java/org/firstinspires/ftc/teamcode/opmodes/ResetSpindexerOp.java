package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
@Disabled
@TeleOp(name = "\uD83D\uDC80 Reset Spindexer", group = "!")
public class ResetSpindexerOp extends CommandOpMode {
    private GamepadEx driver1;
    private DcMotor spindexer;
    private Servo led;
    private ElapsedTime timer;

    @Override
    public void initialize() {
        driver1 = new GamepadEx(gamepad1);
        spindexer = hardwareMap.get(DcMotor.class, "spindexer");
        led = hardwareMap.get(Servo.class, "led");
        timer = new ElapsedTime(); // ✅ initialize timer before use
        super.reset();
    }

    @Override
    public void run() {
        // Reset spindexer encoder when pressing B
        if (gamepad1.b) {
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // 🌈 Smooth rainbow position cycle between 0.3 and 0.722
        double t = timer.seconds();
        double min = 0.3;
        double max = 0.722;
        double amplitude = (max - min) / 2.0;
        double midpoint = (max + min) / 2.0;
        double speed = 4.0; // cycles per second — increase for faster transitions

        // Oscillate servo position smoothly with sine wave
        double position = midpoint + amplitude * Math.sin(2 * Math.PI * speed * t);
        led.setPosition(position);

        telemetry.addData("Press B (circle) to reset spindexer", ":)");
        telemetry.addData("Spindexer Position", spindexer.getCurrentPosition());
        telemetry.addData("LED Position", position);
        telemetry.update();

        super.run();
    }
}
