package paths.constraint;

import geometry.Dist;

public class PathConstraint {
    public final double s;
    public final double value_in;

    public enum ConstraintType {
        VELOCITY,
        ACCELERATION,
        ANGULAR_VELOCITY,
        ANGULAR_ACCELERATION
    }

    public final ConstraintType constraintType;

    public PathConstraint(double s, ConstraintType constraintType, Dist value) {
        this.s = s;
        this.constraintType = constraintType;
        value_in = value.getIn();
    }
}