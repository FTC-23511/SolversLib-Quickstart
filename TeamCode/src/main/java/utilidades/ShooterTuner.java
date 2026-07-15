package utilidades;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class ShooterTuner extends OpMode {

    private DcMotorEx flyWheel, flyWheel1;
    private DcMotorEx fwl;
    private DcMotorEx intake;

    public static double p      = 0.0;
    public static double i      = 0.0;
    public static double d      = 0.0;
    public static double target = 1000;

    private double f       = 0.0;
    private double peakTPS = 0.0;


    private enum Mode { NONE, RAW, PIDF }
    private Mode mode = Mode.NONE;

    private boolean lastA = false;
    private boolean lastX = false;

    private final ElapsedTime timer       = new ElapsedTime();
    private double            lastError   = 0;
    private double            integralSum = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setDirection(DcMotorEx.Direction.FORWARD);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fwl = hardwareMap.get(DcMotorEx.class, "fwl");
        fwl.setDirection(DcMotorEx.Direction.REVERSE);
        fwl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // ── A:
        boolean aNow = gamepad1.a;
        if (aNow && !lastA) {
            if (mode != Mode.RAW) {
                mode    = Mode.RAW;
                peakTPS = 0;              // reset peak para medir desde cero
            } else {
                // Sale de RAW: calcula F y apaga
                if (peakTPS > 0) f = 1.0 / peakTPS;
                setMotors(0);
                mode = Mode.NONE;
            }
        }
        lastA = aNow;

        // ── X:
        boolean xNow = gamepad1.x;
        if (xNow && !lastX) {
            if (mode != Mode.PIDF) {
                mode        = Mode.PIDF;
                lastError   = 0;
                integralSum = 0;
                timer.reset();
            } else {
                setMotors(0);
                mode = Mode.NONE;
            }
        }
        lastX = xNow;

        // Velocidad actual
        double currentTPS = (Math.abs(flyWheel.getVelocity()) + Math.abs(fwl.getVelocity())) / 2.0;

        // Control
        double power = 0;

        switch (mode) {
            case RAW:
                if (currentTPS > peakTPS) peakTPS = currentTPS;
                power = 1.0;
                break;

            case PIDF:
                double dt = timer.seconds();
                if (dt < 0.0001) dt = 0.0001;
                timer.reset();

                double error      = target - currentTPS;
                double derivative = (error - lastError) / dt;

                if (Math.abs(error) < 200) integralSum += error * dt;
                else                       integralSum  = 0;

                lastError = error;

                power = (f * target) + (p * error) + (i * integralSum) + (d * derivative);
                power = Math.max(0, Math.min(1, power));
                break;

            case NONE:
                power = 0;
                break;
        }

        setMotors(power);

        // Intake con RT
        intake.setPower(gamepad1.right_trigger > 0.05 ? 1.0 : 0.0);


        telemetry.addData("Modo",        mode == Mode.RAW  ? "■ RAW  (A para salir)" :
                mode == Mode.PIDF ? "■ PIDF (X para salir)" :
                "□ apagado");
        telemetry.addData("─────────────────────", "");
        telemetry.addData("Target TPS",  "%.1f", target);
        telemetry.addData("Actual TPS",  "%.1f", currentTPS);
        telemetry.addData("Error",       "%.1f", target - currentTPS);
        telemetry.addData("Power",       "%.4f", power);
        telemetry.addData("─────────────────────", "");
        telemetry.addData("Peak TPS",    "%.1f  ← estabiliza antes de salir con A", peakTPS);
        telemetry.addData("F calculada", "%.8f", peakTPS > 0 ? 1.0 / peakTPS : 0.0);
        telemetry.addData("F activo",    "%.8f", f);
        telemetry.addData("─────────────────────", "");
        telemetry.addData("p", p);
        telemetry.addData("i", i);
        telemetry.addData("d", d);
        telemetry.update();
    }

    @Override
    public void stop() {
        setMotors(0);
        intake.setPower(0);
    }

    private void setMotors(double power) {
        flyWheel.setPower(power);
        fwl.setPower(power);
    }
}