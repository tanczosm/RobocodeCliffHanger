package ja;

import ags.utils.dataStructures.trees.thirdGenKD.*;

public class ManhattanDistanceFunction implements DistanceFunction {

    private double[] SITUATION_WEIGHTS;

    public void setSituationWeights (double[] weights)
    {
        this.SITUATION_WEIGHTS = weights;
    }

    @Override
    public double distance(double[] p1, double[] p2) {
        double d = 0;

        for (int i = 0; i < p1.length; i++) {

            double diff = Math.abs(p1[i] - p2[i]);

            d += diff * SITUATION_WEIGHTS[i];

        }

        return d;
    }

    @Override
    public double distanceToRect(double[] point, double[] min, double[] max) {
        double d = 0;

        for (int i = 0; i < point.length; i++) {
            double diff = 0;
            if (point[i] > max[i]) {
                diff = Math.abs(point[i] - max[i]);
            } else if (point[i] < min[i]) {
                diff = Math.abs(point[i] - min[i]);
            }
            d += diff * SITUATION_WEIGHTS[i];
        }

        return d;
    }
}