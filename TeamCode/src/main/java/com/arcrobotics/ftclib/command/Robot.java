package com.arcrobotics.ftclib.command;

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
        com.arcrobotics.ftclib.command.CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link com.arcrobotics.ftclib.command.CommandScheduler} instance
     */
    public void run() {
        com.arcrobotics.ftclib.command.CommandScheduler.getInstance().run();
    }

    /**
     * Schedules {@link com.arcrobotics.ftclib.command.Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        com.arcrobotics.ftclib.command.CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link com.arcrobotics.ftclib.command.Subsystem} objects to the scheduler
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

}
