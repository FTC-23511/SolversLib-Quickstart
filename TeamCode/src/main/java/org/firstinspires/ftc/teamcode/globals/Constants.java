package org.firstinspires.ftc.teamcode.globals;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.geometry.Pose2d;

@Config
public class Constants {
    public enum OpModeType {
        AUTO,
        TELEOP
    }
    public enum AllianceColor {
        RED,
        BLUE
    }

    // Drive
    public static OpModeType OP_MODE_TYPE;
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;
    public static Pose2d END_POSE = new Pose2d();

    public static double TRACK_WIDTH = 11.27362; // Inches
    public static double WHEEL_BASE = 11.50976; // Inches
    public static double MAX_VELOCITY = 7.75 * 12; // Inches/second
    public static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(TRACK_WIDTH/2, WHEEL_BASE/2);
    public static double PINPOINT_TELEOP_POLLING_RATE = 20; // Hertz
    public static PIDFCoefficients SWERVO_PIDF_COEFFICIENTS = new PIDFCoefficients(0.6, 0, 0.2, 0);
    public static double FR_ENCODER_OFFSET = 3.602; // Radians
    public static double FL_ENCODER_OFFSET = 3.753; // Radians
    public static double BL_ENCODER_OFFSET = 0.619; // Radians
    public static double BR_ENCODER_OFFSET = 2.149; // Radians
    public static PIDFCoefficients XY_COEFFICIENTS = new PIDFCoefficients(6, 0, 0.2, 0); // Coefficients for inches
    public static PIDFCoefficients HEADING_COEFFICIENTS = new PIDFCoefficients(5, 0, 0, 0); // Coefficients for radians
    public static double XY_TOLERANCE = 0.25; // Inches
    public static double HEADING_TOLERANCE = 0.05; // Radians
    public static double XY_MIN_OUTPUT = 8; // Inches/second
    public static double HEADING_MIN_OUTPUT = 0.15; // Radians/second

    // Intake
    public static double INTAKE_PIVOT_INTAKE = 0.54;
    public static double INTAKE_PIVOT_READY_INTAKE = 0.00; // unused
    public static double INTAKE_PIVOT_HIGH = 0.85;
    public static double INTAKE_PIVOT_HOLD = 0.45;
    public static double INTAKE_PIVOT_TRANSFER = 0.5;

    public static double INTAKE_FORWARD_SPEED = 1.00;
    public static double INTAKE_REVERSE_SPEED = 0.00; // unused
    public static double INTAKE_HOLD_SPEED = 0.00;

    public static double MIN_DISTANCE_THRESHOLD = 0.00;
    public static double MAX_DISTANCE_THRESHOLD = 1.00;

    // Launcher
    public static double RAMP_ENGAGED = 0.545;
    public static double RAMP_DISENGAGED = 0.455;

    public static double LAUNCHER_FAR_VELOCITY = 0.00;
    public static double LAUNCHER_CLOSE_VELOCITY = 0.00;
    public static double LAUNCHER_HOLD_VELOCITY = 0.00;

    public static double HOOD_FAR = 0.0;
    public static double HOOD_CLOSE = 1.0;

    // Turret
}
