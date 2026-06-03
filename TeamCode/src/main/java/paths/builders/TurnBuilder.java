package paths.builders;

import java.util.ArrayList;
import java.util.List;
import paths.callbacks.Callback;
import paths.movements.Turn;
import geometry.Angle;
import geometry.Pose;

public class TurnBuilder {
    private final Pose startPose;
    private Angle targetHeading = null;
    private final List<Runnable> buildTasks = new ArrayList<>();

    protected TurnBuilder(Pose startPose) {
        this.startPose = startPose;
    }

    public TurnBuilder turnTo(Angle targetHeading) {
        this.targetHeading = targetHeading;
        return this;
    }

    public TurnBuilder addAngularCallback(Angle angle, Runnable action) {
        buildTasks.add(() -> {
            Turn finalTurn = new Turn(startPose, targetHeading);
            Angle startRad = finalTurn.getStartPose().getHeading();
            Angle endRad = finalTurn.getEndPose().getHeading();

            double totalDiff = startRad.getShortestAngularDifferenceTo(endRad).getRad();
            double targetDiff = startRad.getShortestAngularDifferenceTo(angle).getRad();

            if (Math.abs(totalDiff) < 1e-6) {
                if (Math.abs(targetDiff) > 1e-6) {
                    throw new IllegalArgumentException("Callback out of bounds: No rotational distance.");
                }
            } else if ((totalDiff * targetDiff < 0) || (Math.abs(targetDiff) > Math.abs(totalDiff))) {
                throw new IllegalArgumentException("Angular callback outside the sweep range.");
            }
        });
        return this;
    }

    public Turn build() {
        if (targetHeading == null) {
            throw new IllegalStateException("Cannot build Turn: No target heading was specified!");
        }

        Turn finalTurn = new Turn(startPose, targetHeading);

        // Setup internal structures via safe execution tasks
        for (Runnable task : buildTasks) {
            task.run();
        }

        // Execute the native Turn object callback attachment safely
        for (Runnable task : buildTasks) {
            finalTurn.addCallback(new Callback(targetHeading, task));
        }

        return finalTurn;
    }
}