package feedforward;

import androidx.annotation.NonNull;

public class FeedforwardLut {
    private final MotionParameters[] params;

    public FeedforwardLut(MotionParameters[] generatedParams) {
        this.params = generatedParams;
    }

    public MotionParameters getFeedforwardParams(double progression) {
        MotionParameters params1 = params[0];
        MotionParameters params2 = params[1];

        for (int i = 1; i < params.length; i++) {
            if (progression <= params[i].getProgression()) {
                params1 = params[i - 1];
                params2 = params[i];
                break;
            }
        }

        double d1 = params1.getProgression();
        double d2 = params2.getProgression();

        double denominator = d2 - d1;
        if (Math.abs(denominator) < 1e-6) {
            return new MotionParameters(
                    params1.getTangentialVel(),
                    params1.getTangentialAccel(),
                    params1.getAngularVel(),
                    params1.getAngularAccel()
            );
        }

        double t = (progression - d1) / denominator;

        return getFeedforwardParams(params1, t, params2);
    }

    @NonNull
    private static MotionParameters getFeedforwardParams(MotionParameters params1, double t,
                                                         MotionParameters params2) {
        double interpTransVel =
                params1.getTangentialVel() + t * (params2.getTangentialVel() - params1.getTangentialVel());
        double interpTransAccel =
                params1.getTangentialAccel() + t * (params2.getTangentialAccel() - params1.getTangentialAccel());
        double interpAngVel =
                params1.getAngularVel() + t * (params2.getAngularVel() - params1.getAngularVel());
        double interpAngAccel =
                params1.getAngularAccel() + t * (params2.getAngularAccel() - params1.getAngularAccel());

        return new MotionParameters(
                interpTransVel,
                interpTransAccel,
                interpAngVel,
                interpAngAccel
        );
    }
}