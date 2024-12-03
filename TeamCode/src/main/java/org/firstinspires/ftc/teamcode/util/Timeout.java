package org.firstinspires.ftc.teamcode.util;

/**
 * A basic timeout class
 */
public class Timeout {
    private final long startTime;
    private final long duration;

    /**
     * Creates a new timeout
     * @param duration The duration of the timeout in seconds
     */
    public Timeout(int duration) {
        this.duration = secondsToNano(duration);
        this.startTime = System.nanoTime();
    }

    /**
     * Converts seconds to nanoseconds
     * @param seconds The number of seconds
     * @return The number of nanoseconds
     */
    private long secondsToNano(int seconds) {
        return (long) seconds * 1000 * 1000 * 1000;
    }

    private int nanoToSeconds(long nanos) {
        return (int) (nanos / 1000 / 1000 / 1000);
    }

    public long elapsed() {
        return System.nanoTime() - this.startTime;
    }

    public long elapsedSec() {
        return nanoToSeconds(elapsed());
    }

    /**
     * @return true if the timeout has expired
     */
    // I really think that using !expired is more intuitive, so I'm suppressing the warning
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean expired() {
        if (GlobalOpMode.opMode.isStopRequested()) return true;
        return elapsed() > this.duration;
    }

    public static void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
        }
    }
}
