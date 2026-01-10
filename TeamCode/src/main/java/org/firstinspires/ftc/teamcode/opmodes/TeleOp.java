package org.firstinspires.ftc.teamcode.opmodes;

import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motifs.GPP;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motifs.PGP;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motifs.PPG;
import static org.firstinspires.ftc.teamcode.RobotConstants.SHOOTER_ANGLE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.Motifs;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.Arrays;
import java.util.function.Supplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop Field Centric", group = "!")
public class TeleOp extends CommandOpMode {


    //Constants
    public enum Alliance {
        RED,
        BLUE
    }
    public enum IntakeState {
        STOP, FORWARD, REVERSE
    }
    final Pose GOAL_RED = new Pose(135,141.5);
    final Pose GOAL_BLUE = new Pose(9,141.5);

    //State variables
    Alliance alliance = Alliance.RED;
    Motifs currentMotif = PPG;
    IntakeState intakeState = IntakeState.STOP;
    boolean manualControl = true;
    boolean slowMode = false;

    //subsystems
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorSensors;
    private LEDSubsystem led;
    private GateSubsystem gate;
    private LimelightSubsystem limelight;
    public VoltageSensor voltageSensor;
    public GamepadEx driver1;
    public GamepadEx driver2;

    //Pedro and PID
    private Follower follower;
    public static Pose startingPose;
    public static Pose savedPose = new Pose(0,0,0);
    private Supplier<PathChain> pathChainSupplier;
    //Auto aligner
    public static double alignerHeadingkP = -1.0; //Coefficients copied from pedro pathing.
    public static double alignerHeadingkD = 0.02;
    public static double alignerHeadingkF = 0.01;
    PIDFController alignerHeadingPID = new PIDFController(alignerHeadingkP, 0, alignerHeadingkD, alignerHeadingkF);
    double lastSeenX;
    double headingVector;

    //Voltage compensation
    double currentVoltage = 14;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();

    //Timer
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

    @Override
    public void initialize () {
        initializeSystems();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        createBinds();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        handleTeleopDrive();
        handleLED();
        handleVoltageCompensation();
        handleSpindexer();

        //Update color sensors
        colorSensors.updateSensor1();
        colorSensors.updateSensor2();
        colorSensors.updateBack(); //Update every time.... for now .......

        handleTelemetry();

        follower.update();
        loopTimer.reset();
        telemetry.update();
        super.run();


    }
    void initializeSystems() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(1.0);
        startingPose = (Pose) blackboard.getOrDefault("endpose", new Pose(0,0,0));
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorSensors = new ColorSensorsSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        spindexer.set(75);
        gate.down();

        register(intake, shooter, spindexer, gate, colorSensors, led, limelight);
        super.reset();
        lastVoltageCheck.reset();
        follower.startTeleopDrive();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
    }
    void createBinds() {
        //Driver 1
        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.FORWARD) intakeState = IntakeState.STOP;
                    else intakeState = IntakeState.FORWARD;
                    new SelectCommand(this::getIntakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.REVERSE) intakeState = IntakeState.STOP;
                    else intakeState = IntakeState.REVERSE;
                    new SelectCommand(this::getIntakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new MoveSpindexerCommand(spindexer, gate, 1, true)
        );
        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new MoveSpindexerCommand(spindexer, gate, -1, true)
        );


        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));

        //Driver 2
        //TODO: add ability to switch between shooting in motif order and shooting in any order (motif XXX)

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> {
                    if (currentMotif == PPG) {
                        currentMotif = GPP;
                    }
                    else if (currentMotif == GPP) {
                        currentMotif = PGP;
                    }
                    else if (currentMotif == PGP) {
                        currentMotif = PPG;
                    }
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> {
                    if (currentMotif == PPG) {
                        currentMotif = PGP;
                    }
                    else if (currentMotif == PGP) {
                        currentMotif = GPP;
                    }
                    else if (currentMotif == GPP) {
                        currentMotif = PPG;
                    }
                })
        );
        new Trigger(
                () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) //far distance
                .whileActiveContinuous(new InstantCommand(() -> {
                            manualControl = false;
                        })
                );
        driver2.getGamepadButton(LEFT_BUMPER).whenActive(  //turn off shooter
                new InstantCommand(() -> {
                    shooter.setTargetLinearSpeed(0);
                    gamepad2.rumbleBlips(1);
                })
        );
        driver2.getGamepadButton(LEFT_BUMPER).whenActive(
                new InstantCommand(() -> {
                    follower.setPose(follower.getPose().setHeading(0));
                })
        );
    }
    void handleTeleopDrive() {
        //Drivetrain code
        if (manualControl) {
            double x = -driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = -driver1.getRightX() * (slowMode?0.3:1);
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
            follower.setTeleOpDrive(x / denominator, y / denominator, rx / denominator, false);
        } else {
            if (gamepad1.touchpad_finger_1) {
                manualControl = true;
                gamepad1.rumbleBlips(1);
            }
            double x = -driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = 0;
            Vector v_ball = calculateTargetVector2(follower, (alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE), shooter);
            double targetDirection = v_ball.getTheta();
            double error = getAngleDifference(targetDirection, follower.getHeading());
            rx = alignerHeadingPID.calculate(error, 0);
            shooter.setTargetLinearSpeed(v_ball.getMagnitude());
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
            follower.setTeleOpDrive(y / denominator, x / denominator, rx / denominator, false);
        }
    }
    void handleLED() {
        //LED Code
        if (intakeState == IntakeState.FORWARD) {
            led.setColor(LEDSubsystem.LEDState.WHITE);
        }
        else if (shooter.getActualVelocity() > 300) { //shooting mode
            if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -30) {
                led.setColor(LEDSubsystem.LEDState.RED);
            }
            else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
                led.setColor(LEDSubsystem.LEDState.BLUE);
            }
            else {
                led.setColor(LEDSubsystem.LEDState.GREEN);
            }
        }
        else {
            double t = totalTimer.seconds();
            double min = 0.3;
            double max = 0.722;
            double amplitude = (max - min) / 2.0;
            double midpoint = (max + min) / 2.0;
            double speed = 0.6; // cycles per second — increase for faster transitions

            // Oscillate servo position smoothly with sine wave
            double position = midpoint + amplitude * Math.sin(2 * Math.PI * speed * t);
            led.setPosition(position);
        }
    }
    void handleVoltageCompensation() {
        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }
    }
    void handleSpindexer() {
        //spindexer and array logic
        if ((Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60)) {
            spindexer.handleUpdateArray(colorSensors.getIntakeSensor1Result(), colorSensors.getIntakeSensor2Result(), colorSensors.getBackResult());
            if (colorSensors.doesLastResultHaveBall() && spindexer.getBalls()[2].equals(RobotConstants.BallColors.NONE)) {
                schedule(new MoveSpindexerCommand(spindexer, gate, 1, true));
            }
        }
    }
    void handleTelemetry() {
        telemetry.addLine(alliance == Alliance.RED ? "\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34" : "\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35");
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.addData("Mode", manualControl ? "Manual" : "Auto-Aim");
        telemetry.addData("Selected Motif", currentMotif);
        telemetry.addData("Balls Array", Arrays.toString(spindexer.getBalls()));
        telemetry.addLine("--Spindexer--");
        telemetry.addData("PID output", spindexer.getOutput());
        telemetry.addData("PID setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("Unwrapped position", spindexer.getCurrentPosition());
        telemetry.addLine("--Shooter--");
        telemetry.addData("Target velocity", shooter.getTargetVelocity());
        telemetry.addData("Actual velocity ", shooter.getActualVelocity());
        telemetry.addData("Linear speed ", shooter.getFlywheelLinearSpeed());
        telemetry.addLine("--Color Sensors--");
        NormalizedRGBA val1 = colorSensors.getIntakeSensor1Result();
        NormalizedRGBA val2 = colorSensors.getIntakeSensor2Result();
        NormalizedRGBA valBack = colorSensors.getBackResult();
        // -- Sensor 1 (Intake) --
        String color1 = "None";
        float[] hsv1 = {0,0,0};
        if (val1 != null) {
            hsv1 = ColorSensorsSubsystem.rgbToHsv(val1);
            if (ColorSensorsSubsystem.colorIsPurpleIntake(val1)) color1 = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenIntake(val1)) color1 = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(val1)) color1 = "White";
        }
        telemetry.addData("Intake 1", "[%s] H:%.0f S:%.2f V:%.2f", color1, hsv1[0], hsv1[1], hsv1[2]);
        // -- Sensor 2 (Intake) --
        String color2 = "None";
        float[] hsv2 = {0,0,0};
        if (val2 != null) {
            hsv2 = ColorSensorsSubsystem.rgbToHsv(val2);
            if (ColorSensorsSubsystem.colorIsPurpleIntake(val2)) color2 = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenIntake(val2)) color2 = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(val2)) color2 = "White";
        }
        telemetry.addData("Intake 2", "[%s] H:%.0f S:%.2f V:%.2f", color2, hsv2[0], hsv2[1], hsv2[2]);
        // -- Back Sensor (Uses Back-specific logic) --
        String colorBack = "None";
        float[] hsvBack = {0,0,0};
        if (valBack != null) {
            hsvBack = ColorSensorsSubsystem.rgbToHsv(valBack);
            if (ColorSensorsSubsystem.colorIsPurpleBack(valBack)) colorBack = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenBack(valBack)) colorBack = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(valBack)) colorBack = "White";
        }
        telemetry.addData("Back", "[%s] H:%.0f S:%.2f V:%.2f", colorBack, hsvBack[0], hsvBack[1], hsvBack[2]);
        telemetry.addLine("--Pedro--");
        telemetry.addData("Position ", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("Heading ", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("Slow mode", slowMode);

    }
    /**
     * Calculate the target vector for the shooter with velocity compensation.
     * @param follower Pedro follower object
     * @param targetPose Target coordinate in Pedro system to shoot at.
     * @param shooter shooter subsystem
     * @return A vector representing the trajectory the ball should follow. The magnitude of the vector is the linear speed the ball should have.
     */
    public Vector calculateTargetVector(Follower follower, Pose targetPose, ShooterSubsystem shooter) {
        Pose robotPose = follower.getPose();
        Vector v_robot = follower.getVelocity(); //Assume its inches per second. in polar
        Pose robotFuturePose = robotPose.plus(new Pose(v_robot.getXComponent(), v_robot.getYComponent()).times(0.200)); //200 ms latency for shooter
        double dx = targetPose.getX() - robotFuturePose.getX();
        double dy = targetPose.getY() - robotFuturePose.getY();
        double speed = shooter.findSpeedFromDistance(Math.hypot(dx, dy));
        double horizontalSpeed = speed * Math.cos(SHOOTER_ANGLE);
        double idealHeading = Math.atan2(dy, dx);
        Vector v_target = new Vector(speed, idealHeading); //is polar
        Vector v_ball = v_target.minus(v_robot);
        return new Vector(v_ball.getMagnitude(), v_ball.getTheta() / Math.cos(SHOOTER_ANGLE));
    }

    /**
     * Calculate the target vector for the ball with velocity and acceleration compensation, as well as latency in the shooter.
     * @param follower Pedro follower object
     * @param targetPose Target coordinate in Pedro system to shoot at.
     * @param shooter shooter subsystem
     * @return A vector representing the trajectory the ball should follow. The magnitude of the vector is the linear speed the ball should have.
     */
    public Vector calculateTargetVector2(Follower follower, Pose targetPose, ShooterSubsystem shooter) {
        // --- 0. CONFIGURATION ---
        // You must estimate your shooter's launch angle relative to the floor.
        // If your hood moves, calculate this based on hood position.
        // For fixed hoods, 45-60 degrees is common.
        double launchAngle = SHOOTER_ANGLE;
        double latency = 0.300; //TODO: MEasure

        // --- 1. GATHER CURRENT STATE ---
        Pose currentPose = follower.getPose();
        Vector v_robot = follower.getVelocity();
        double angularVel = follower.getAngularVelocity();
        Vector a_robot = follower.getAcceleration();

        double shooterOffsetX = 10.0; //TODO: Measure
        double shooterOffsetY = 0.0;

        // --- 2. PREDICT ROBOT POSE (Standard Kinematics) ---
        double futureHeading = currentPose.getHeading() + (angularVel * latency);

        // Position Prediction
        double predX = currentPose.getX() + (v_robot.getXComponent() * latency) + (0.5 * a_robot.getXComponent() * latency * latency);
        double predY = currentPose.getY() + (v_robot.getYComponent() * latency) + (0.5 * a_robot.getYComponent() * latency * latency);

        // --- 3. CALCULATE ROBOT VELOCITY AT MUZZLE (Standard Rigid Body) ---
        double futureVx = v_robot.getXComponent() + (a_robot.getXComponent() * latency);
        double futureVy = v_robot.getYComponent() + (a_robot.getYComponent() * latency);

        double cosH = Math.cos(futureHeading);
        double sinH = Math.sin(futureHeading);

        // Rotated offset
        double fieldOffsetX = (shooterOffsetX * cosH) - (shooterOffsetY * sinH);
        double fieldOffsetY = (shooterOffsetX * sinH) + (shooterOffsetY * cosH);

        // Tangential velocity
        double v_tangential_x = -angularVel * fieldOffsetY;
        double v_tangential_y =  angularVel * fieldOffsetX;

        // Total Robot Velocity Components (Cartesian)
        double finalRobotVx = futureVx + v_tangential_x;
        double finalRobotVy = futureVy + v_tangential_y;

        // Convert to Polar for Pedro Vector
        double robotVelMag = Math.hypot(finalRobotVx, finalRobotVy);
        double robotVelAngle = Math.atan2(finalRobotVy, finalRobotVx);
        Vector v_robot_total = new Vector(robotVelMag, robotVelAngle);

        // --- 4. SOLVE FOR SHOOTING VECTOR ---
        double shooterMsgX = predX + fieldOffsetX;
        double shooterMsgY = predY + fieldOffsetY;

        double dx = targetPose.getX() - shooterMsgX;
        double dy = targetPose.getY() - shooterMsgY;
        double dist = Math.hypot(dx, dy);
        double idealHeading = Math.atan2(dy, dx);

        // === THE FIX STARTS HERE ===

        // A. Get the Total Exit Speed required for this distance (from your lookup table/regression)
        double totalSpeedRequired = shooter.findSpeedFromDistance(dist);

        // B. "Flatten" this speed to the 2D floor plane
        //    We only want the horizontal component for vector math
        double horizontalSpeed = totalSpeedRequired * Math.cos(launchAngle);

        // C. Create the target vector using Horizontal Speed
        Vector v_target_horizontal = new Vector(horizontalSpeed, idealHeading);

        // D. Perform Vector Subtraction in the 2D plane
        //    (Horizontal Target) - (Horizontal Robot Motion) = (Horizontal Ball Launch Vector)
        Vector v_ball_horizontal = v_target_horizontal.minus(v_robot_total);

        // E. Convert the result back to Total Exit Speed for the flywheel
        //    Total = Horizontal / cos(theta)
        double finalHorizontalSpeed = v_ball_horizontal.getMagnitude();
        double finalTotalSpeed = finalHorizontalSpeed / Math.cos(launchAngle);

        // Return a Vector with the NEW Total Speed and the CORRECTED heading
        return new Vector(finalTotalSpeed, v_ball_horizontal.getTheta());
    }
    /**
     * Calculates the smallest difference between two angles in radians.
     * @return a double between -PI and +PI.
     */
    public double getAngleDifference(double targetAngle, double currentAngle) {
        double difference = targetAngle - currentAngle;

        // Normalize the angle to be within -PI to +PI
        while (difference > Math.PI) {
            difference -= 2 * Math.PI;
        }
        while (difference <= -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    }
    public Command getIntakeCommand() {
        switch (intakeState) {
            case FORWARD:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKING);
                });
            case REVERSE:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.REVERSE);
                });
            case STOP:
            default:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.STILL);
                });
        }
    }
}