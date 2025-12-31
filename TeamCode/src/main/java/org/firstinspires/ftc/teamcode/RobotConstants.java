package org.firstinspires.ftc.teamcode;

public class RobotConstants {
    public static final double SPINDEXER_COUNTS_PER_REVOLUTION = 4096;
    public static final double SPINDEXER_TICKS_PER_DEG = SPINDEXER_COUNTS_PER_REVOLUTION / 360.0; // CPR 8192

    public static final double SHOOTER_ANGLE = 60; //TODO: Measure
    public enum BallColors {
        NONE, PURPLE, GREEN, UNKNOWN
    }

    public enum Motifs {
        PPG, PGP, GPP, PPP
    }

}
