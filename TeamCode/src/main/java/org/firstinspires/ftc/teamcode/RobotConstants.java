package org.firstinspires.ftc.teamcode;

public class RobotConstants {
    //INTAKE
    public static double INTAKE_IN = 1.0;

    //SHOOTER
    public static double SHOOTER_CACHETHRESHOLD = 0.0005;
    public static double SHOOTER_ON = 0.7; //expressed as percentage of max voltage
    //SPINDEXER
    public static double SPINDEXER_CACHETHRESHOLD = 0.001;
    public static double SPINDEXER_COUNTS_PER_REVOLUTION = 4096;
    public static double SPINDEXER_TICKS_PER_DEG = SPINDEXER_COUNTS_PER_REVOLUTION / 360.0; // CPR 8192

    public static int SPINDEXER_INITPOS = -(int) SPINDEXER_TICKS_PER_DEG * 100;

    public enum BallColors {
        NONE, PURPLE, GREEN, UNKNOWN
    }

    public enum Motifs {
        PPG, PGP, GPP, PPP
    }

}
