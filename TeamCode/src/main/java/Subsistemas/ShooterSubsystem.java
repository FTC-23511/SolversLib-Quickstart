package Subsistemas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterSubsystem extends SubsystemBase {

    public static PIDFCoefficients SHOOTER_COEFFS = new PIDFCoefficients(0.0085, 0, 0, 0);
    public static double kV = 0.000455;

    private final DcMotorEx flywheel1;
    private final DcMotorEx flywheel2;

    private final PIDFController shooterPIDF;
    private double currentTarget = 0;
    private boolean enabled = false;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterPIDF = new PIDFController(SHOOTER_COEFFS);

    }

    @Override
    public void periodic() {
        if (enabled) {
            shooterPIDF.setCoefficients(SHOOTER_COEFFS);
            shooterPIDF.setSetPoint(currentTarget);

            double currentVel = flywheel2.getVelocity();
            double power = (kV * currentTarget) + shooterPIDF.calculate(currentVel);
            power = Math.max(-1.0, Math.min(1.0, power));

            flywheel1.setPower(power);
            flywheel2.setPower(power);
        } else {
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }

        // Telemetría
    }

    public void setTargetVelocity(double velocity) {
        this.currentTarget = velocity;
        enabled = true;
    }

    public void stop() {
        enabled = false;
        currentTarget = 0;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(currentTarget - flywheel2.getVelocity()) <= tolerance;
    }
}