package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

@TeleOp (name = "\uD83D\uDC80 Reset Spindexer", group = "OpModes")
public class ResetSpindexerOp extends CommandOpMode {
    private GamepadEx driver1;
    private DcMotor spindexer;


    @Override
    public void initialize () {
        //systems and pedro
        driver1 = new GamepadEx(gamepad1);
        spindexer = hardwareMap.get(DcMotor.class, "spindexer");

        super.reset();
    }

    @Override
    public void run() {
        if (gamepad1.b) {
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addData("Press circle to reset spindexer.", ":)");
        telemetry.addData("Spindexer Position", spindexer.getCurrentPosition());
        telemetry.update();
        super.run();

    }
}