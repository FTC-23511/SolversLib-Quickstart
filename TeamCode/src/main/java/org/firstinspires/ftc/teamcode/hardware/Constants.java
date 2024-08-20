package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class Constants {
    // By default values refer to servo positions, unless otherwise specified
    // By default for values that control opposite running hardware, the right value of the hardware is used
    // e.g. for ARM_TRANSFER_POS, it should correspond with the real position of the rightArm servo at the transfer

    // Deposit
    public static double ARM_TRANSFER_POS = 0;
    public static double ARM_BACKDROP_POS = 0;

    public static double WRIST_TRANSFER_POS = 0;
    // 4th item or 3rd with 0-index is always the default (middle horizontal)
    public static double[] WRIST_BACKDROP_POSITIONS = {1, 0.82, 0.64, 0.46, 0.28, 0.08};
    public static double LEFT_CLAW_OPEN_POS = 0.07;
    public static double LEFT_CLAW_CLOSE_POS = 0.18;
    public static double RIGHT_CLAW_OPEN_POS = 0.59;
    public static double RIGHT_CLAW_CLOSE_POS = 0.74;

    // Slides
    // Encoder ticks for max extension for extendo
    public static double MAX_EXTENDO_EXTENSION = 0; // Encoder ticks
    // Encoder ticks for max extension on scoring slides
    public static double MAX_SLIDES_EXTENSION = 10000; // Encoder ticks
    // Encoder ticks for first pixel row height
    public static double FIRST_BACKDROP_ROW = 0; // Encoder ticks
    // Encoder ticks for slides between pixel row heights on backdrop
    public static double BACKDROP_INCREMENTAL_HEIGHT = 0; // Encoder ticks

    // Intake
    public static double INTAKE_POWER = 0.5; // Motor power
    public static double INTAKE_REVERSE_POWER = 0.5; // Motor power
    // Highest to lowest (lowest being normal intake flat on ground) servo positions for pitching intake
    // 0th index is fully retracted
    public static double[] STACK_HEIGHTS = {};
    public static double TRAY_INTAKE = 0;
    public static double TRAY_TRANSFER = 0.55;
    // Time for distance sensor to detect a pixel until it knows it has 2 pixels in milliseconds
    public static double TWO_PIXEL_TIME = 1000;
    // Distance for distance sensor to return as lower than for a pixel to be detected in tray in centimeters
    public static double PIXEL_DISTANCE = 5;
}
