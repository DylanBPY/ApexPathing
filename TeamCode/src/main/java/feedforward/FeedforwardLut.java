package feedforward;

import androidx.annotation.NonNull;

public class FeedforwardLut {
    private final FeedforwardParams[] params;

    public FeedforwardLut(FeedforwardParams[] generatedParams) {
        this.params = generatedParams;
    }

    public FeedforwardParams getFeedforwardParams(double distAlongCurve) {
        FeedforwardParams params1 = params[0];
        FeedforwardParams params2 = params[1];

        for (int i = 1; i < params.length; i++) {
            if (distAlongCurve <= params[i].getDistAlongCurve()) {
                params1 = params[i - 1];
                params2 = params[i];
                break;
            }
        }

        double d1 = params1.getDistAlongCurve();
        double d2 = params2.getDistAlongCurve();

        double denominator = d2 - d1;
        if (Math.abs(denominator) < 1e-6) {
            return new FeedforwardParams(
                    params1.getTangentialVel(),
                    params1.getTangentialAccel(),
                    params1.getAngularVel(),
                    params1.getAngularAccel()
            );
        }

        double t = (distAlongCurve - d1) / denominator;

        FeedforwardParams interpolated = getFeedforwardParams(params1, t, params2);

        return interpolated;
    }

    @NonNull
    private static FeedforwardParams getFeedforwardParams(FeedforwardParams params1, double t, FeedforwardParams params2) {
        double interpTransVel = params1.getTangentialVel() + t * (params2.getTangentialVel() - params1.getTangentialVel());
        double interpTransAccel = params1.getTangentialAccel() + t * (params2.getTangentialAccel() - params1.getTangentialAccel());
        double interpAngVel = params1.getAngularVel() + t * (params2.getAngularVel() - params1.getAngularVel());
        double interpAngAccel = params1.getAngularAccel() + t * (params2.getAngularAccel() - params1.getAngularAccel());

        return new FeedforwardParams(
                interpTransVel,
                interpTransAccel,
                interpAngVel,
                interpAngAccel
        );
    }
}