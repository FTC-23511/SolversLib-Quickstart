package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.Contract;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class System {
    // Example Usage for gamepad1 right bumper: checkButton(currentGamepad1, "right_bumper")
    public static boolean checkButton(Gamepad gamepad, String button) {
        try {
            String buttons = String.valueOf(gamepad).substring(75).substring(1);
            return !buttons.contains(button);
        }
        catch (Exception ignored) {
            return (true);
        }
    }

    public static double round(double number, int places) {
        return new BigDecimal((String.valueOf(number))).setScale(places, RoundingMode.HALF_UP).doubleValue();
    }

    /**
     * Do the angle optimization for swerve.
     * @return The optimized angle.
     */
    public static double optimize(double angle, double goalAngle) {
        if (System.canBeOptimized(angle, goalAngle)) {
            return System.angleWrap(angle - Math.PI);
        }
        return angle;
    }

    public static boolean canBeOptimized(double angle, double goalAngle) {
        double error = System.angleWrap(goalAngle - angle);
        return Math.abs(error) > Math.PI / 2;
    }

    /**
     * Change an angle to be between 0 and 360 degrees.
     * @param angle An angle, in radians.
     * @return The normalized angle.
     */
    public static double normalize(double angle) {
        while (angle > 2*Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Gets the difference between two angles.
     * @param angle1 The first angle
     * @param angle2 The second angle
     * @return The difference between the two angles, in radians
     */
    public static double getdelta(double angle1, double angle2){ // function for difference between 2 angles
        double[] changes = {-2.0*Math.PI, 0.0, 2.0*Math.PI}; // add/subtract 2pi radians
        double min_delta = Math.abs(angle2 - angle1);
        for (int i=0; i < 3; i++) {
            min_delta = Math.min(min_delta, Math.abs((angle2 + changes[i]) - angle1));
        }
        return min_delta;
    }

    public static double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public static class QuadraticFormulaResult {
        public final int answerCount;
        public final double root1;
        public final double root2;

        public QuadraticFormulaResult(double root1, double root2, int answerCount) {
            this.answerCount = answerCount;
            this.root1 = root1;
            this.root2 = root2;
        }
    }

    public static int sign(double num) {
        if (num >= 0) {
            return 1;
        } else {
            return -1;
        }
    }

    /**
     * Solve for all real solutions to a quadratic.
     * @param a The coefficient of the x^2 term.
     * @param b The coefficient of the x term.
     * @param c The final term.
     * @return All real solutions to the quadratic.
     */
    @NonNull
    @Contract(value = "_, _, _ -> new", pure = true)
    public static QuadraticFormulaResult quadraticFormula(double a, double b, double c) {
        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) { // no real slns
            return new QuadraticFormulaResult(0, 0, 0);
        }

        double ans1 = (-b + Math.sqrt(discriminant)) / (2*a);
        if (discriminant == 0) { // one real sln
            return new QuadraticFormulaResult(ans1, ans1, 1);
        }

        double ans2 = (-b - Math.sqrt(discriminant)) / (2*a);
        return new QuadraticFormulaResult(ans1, ans2, 2);
    }
}