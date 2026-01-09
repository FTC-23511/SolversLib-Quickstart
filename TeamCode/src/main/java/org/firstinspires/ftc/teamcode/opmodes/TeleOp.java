package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.function.Supplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop Field Centric", group = "!")
public class TeleOp extends CommandOpMode {
    public Motifs motifs = PPG;
    public enum Alliance {
        RED,
        BLUE
    }
    public Alliance alliance = Alliance.RED;
    public final Pose GOAL_RED = new Pose(135,141.5);
    public final Pose GOAL_BLUE = new Pose(9,141.5);

    //pedro
    private Follower follower;
    public static Pose startingPose;
    public static Pose savedPose = new Pose(0,0,0);
    private Supplier<PathChain> pathChainSupplier;
    private double fieldOffset = 0;  // degrees


    //subsystems
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorSensors;
    private LEDSubsystem led;
    private GateSubsystem gate;
    private LimelightSubsystem limelight;

    //gamepads
    public GamepadEx driver1;
    public GamepadEx driver2;

    //vision
    boolean cameraInitialized = false;

    //autodrive
    private boolean manualControl = true;

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();

    //true = controlling far
    boolean isAdjustingFar;


    //looptime
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime totalTimer = new ElapsedTime();

    //intake state machine
    public enum IntakeState {
        STOP, FORWARD, REVERSE
    }
    private IntakeState intakeState = IntakeState.STOP;
    public Command intakeCommand() {
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


    //point to april tag
    public static double headingkP = -1.0; //Coefficients copied from pedro pathing.
    public static double headingkD = 0.02;
    public static double headingkF = 0.01;
    PIDFController headingPID = new PIDFController(headingkP, 0, headingkD, headingkF);
    double lastSeenX;
    double headingVector;

    //color array and spindexer moving
    boolean spindexerBusy;
    boolean ball = false;

    @Override
    public void initialize () {
        //systems and pedro
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

        super.reset();
        lastVoltageCheck.reset();
        register(intake, shooter, spindexer, gate, colorSensors, led, limelight);

        spindexer.set(75);
        gate.down();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //pedro and gamepad wrapper
        follower.startTeleopDrive();
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
//        pathChainSupplier = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, savedPose)))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, savedPose::getHeading, 0.8))
//                .build();
        //command binding
        SelectCommand intakeSelectCommand = new SelectCommand(this::intakeCommand);

        //Overview of controls:
        //D1 controls drivetrain, intake and shooter/snap to angle. Spindexer shuold automatically advance when color sensor reads something
        //Driver 1
        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.FORWARD) intakeState = IntakeState.STOP;
                    else intakeState = IntakeState.FORWARD;
                    new SelectCommand(this::intakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.REVERSE) intakeState = IntakeState.STOP;
                    else intakeState = IntakeState.REVERSE;
                    new SelectCommand(this::intakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {spindexer.moveSpindexerBy(120);})
        );
        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {spindexer.moveSpindexerBy(-120);})
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
                    if (motifs == PPG) {
                        motifs = GPP;
                    }
                    else if (motifs == GPP) {
                        motifs = PGP;
                    }
                    else if (motifs == PGP) {
                        motifs = PPG;
                    }
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> {
                    if (motifs == PPG) {
                        motifs = PGP;
                    }
                    else if (motifs == PGP) {
                        motifs = GPP;
                    }
                    else if (motifs == GPP) {
                        motifs = PPG;
                    }
                })
        );
        new Trigger(
                () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) //far distance
                    .whileActiveContinuous(new InstantCommand(() -> {
                        manualControl = false;
                    })
                );
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(  //turn off shooter
                new InstantCommand(() -> {
                    shooter.setTargetLinearSpeed(0);
                    gamepad2.rumbleBlips(1);
                })
        );
        //TODO: driver 2 reset heading to 0

    }

    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        gate.down(); //temp fix

        //Drivetrain code
        if (manualControl) {

            double x = -driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = -driver1.getRightX() * (slowMode?0.3:1);
            // rotate driver input by the fieldOffset
            double rotatedX =  x * Math.cos(fieldOffset) - y * Math.sin(fieldOffset);
            double rotatedY =  x * Math.sin(fieldOffset) + y * Math.cos(fieldOffset);

            double denominator = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(rx), 1.0);


            // send rotated inputs
            follower.setTeleOpDrive(rotatedY / denominator, rotatedX / denominator, rx / denominator, false);

        } else {
            if (gamepad1.touchpad_finger_1) {
                manualControl = true;
                gamepad1.rumbleBlips(1);
            }
            double x = -driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = 0;
            double rotatedX =  x * Math.cos(fieldOffset) - y * Math.sin(fieldOffset);
            double rotatedY =  x * Math.sin(fieldOffset) + y * Math.cos(fieldOffset);
            Vector v_ball = calculateTargetVector2(follower, (alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE), shooter);
            double targetDirection = v_ball.getTheta();
            double error = getAngleDifference(targetDirection, follower.getHeading());
            rx = headingPID.calculate(error, 0);
            shooter.setTargetLinearSpeed(v_ball.getMagnitude());
            double denominator = Math.max(Math.abs(rotatedX) + Math.abs(rotatedY) + Math.abs(rx), 1.0);
            follower.setTeleOpDrive(rotatedY / denominator, rotatedX / denominator, rx / denominator, false);
        }
        follower.update();

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

        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }

        //spindexer and array logic
        if ((Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60)) {
            spindexer.handleUpdateArray(colorSensors.getIntakeSensor1Result(), colorSensors.getIntakeSensor2Result(), colorSensors.getBackResult());
            if (colorSensors.doesLastResultHaveBall() && spindexer.getBalls()[2].equals(RobotConstants.BallColors.NONE)) {
                spindexer.moveSpindexerBy(120);
            }
        }

//        telemetry.addData("BALLS", Arrays.toString(spindexer.getBalls()));

        telemetry.addData("Loop Time", timer.milliseconds());

//        telemetry.addData("current motif ", motifs);
        telemetry.addData("spindexer output ", spindexer.getOutput());
        telemetry.addData("spindexer setpoint ", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos ", spindexer.getCurrentPosition());
        telemetry.addData("is spindexer ready to read color ", spindexer.availableToSenseColor());

        telemetry.addData("------------------","");

        telemetry.addData("last seen goal x pos ", lastSeenX);
        telemetry.addData("last pid power to heading", headingVector);

        telemetry.addData("------------------","");
        telemetry.addData("shooter target velocity ", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity ", shooter.getActualVelocity());
        telemetry.addData("shooter linear speed ", shooter.getFlywheelLinearSpeed());

        telemetry.addData("------------------","");

        telemetry.addData("current pos ", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading ", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("t value ", follower.getCurrentTValue());
        telemetry.addData("slowmode ", slowMode);
        telemetry.addData("------------------","");
//        float[] hsv1 = colorSensors.senseColorsHSV(1);
//        float[] hsv2 = colorSensors.senseColorsHSV(2);
//
//        String color1 = "none";
//        String color2 = "none";
//
//        // Sensor 1
//        if (ColorSensorsSubsystem.colorIsPurple(hsv1)) {
//            color1 = "purple";
//        } else if (ColorSensorsSubsystem.colorIsGreen(hsv1)) {
//            color1 = "green";
//        } else if (ColorSensorsSubsystem.colorIsWhite(hsv1)) {
//            color1 = "white";
//        }
//
//        // Sensor 2
//        if (ColorSensorsSubsystem.colorIsPurple(hsv2)) {
//            color2 = "purple";
//        } else if (ColorSensorsSubsystem.colorIsGreen(hsv2)) {
//            color2 = "green";
//        } else if (ColorSensorsSubsystem.colorIsWhite(hsv2)) {
//            color2 = "white";
//        }
//
//
//
//        // ONE telemetry block, no ifs
//        telemetry.addData("Sensor 1 Left", color1 + " | raw: " + Arrays.toString(hsv1));
//        telemetry.addData("Sensor 2 Right", color2 + " | raw: " + Arrays.toString(hsv2));


        timer.reset();
        telemetry.update();
        super.run();


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
}