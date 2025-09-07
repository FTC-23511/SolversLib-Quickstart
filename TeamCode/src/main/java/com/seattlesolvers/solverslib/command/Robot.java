package com.seattlesolvers.solverslib.command;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This is the Robot class. This will make your command-based robot code a lot smoother
 * and easier to understand.
 */
public class Robot {

    public static boolean isDisabled = false;

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules {@link Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link Subsystem} objects to the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    public static void disable() {
        isDisabled = true;
    }

    public static void enable() {
        isDisabled = false;
    }

    /**
     * Method to automatically set all hubs to bulk read, greatly reducing loop times.
     * @param hwMap hardwareMap to access hub objects
     * @param cachingMode the mode in which the hubs operate during bulk reading/caching.
     *                    MANUAL mode is highly recommended and comes to the user with no
     *                    extra work and doesn't read any hardware more than once per loop,
     *                    while AUTO enables bulk reads but will conduct a bulk read any time
     *                    a specific hardware is read the second time, even in a loop,
     *                    potentially leading to worse loop times.
     */
    public void setBulkReading(HardwareMap hwMap, LynxModule.BulkCachingMode cachingMode) {
        CommandScheduler.getInstance().setBulkReading(hwMap, cachingMode);
    }

}
