package paths;

/**
 * Represents a scheduled action that executes at a specific progression point along a path segment.
 * <p>
 * Callbacks are used to trigger robot mechanisms (e.g., opening a claw, moving an arm)
 * dynamically as the robot drives, without pausing the path execution. The execution point
 * is defined by the parameter 's', which represents the physical distance percentage [0.0, 1.0]
 * along the segment.
 * @author DrPixelCat
 */
public class Callback {
    private final double s;
    private final Runnable runnable;
    private boolean triggered = false;

    /**
     * Constructs a new Callback attached to a specific progression percentage.
     * * @param s        The physical distance percentage [0.0, 1.0] along the segment at which to trigger the callback.
     * @param runnable The code/action to execute when the progression target is reached.
     */
    public Callback(double s, Runnable runnable) {
        this.runnable = runnable;
        this.s = s;
    }

    /**
     * Retrieves the executable action associated with this callback.
     * * @return The {@link Runnable} action.
     */
    public Runnable getRunnable() {
        return runnable;
    }

    /**
     * Retrieves the target progression percentage for this callback.
     * * @return The distance percentage 's' [0.0, 1.0].
     */
    public double getS() {
        return s;
    }

    /**
     * Marks this callback as triggered to prevent it from executing multiple times
     * during the same path execution.
     */
    public void trigger() {
        triggered = true;
    }
}