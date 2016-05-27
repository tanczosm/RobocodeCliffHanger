package ja;

/**
 * Created by eahscs on 5/25/2016.
 */
public class Situation {

    public static double[] SITUATION_WEIGHTS = {1.0, 0.5, 1.0, 1.0, 1.0};

    public long time;
    public double distance;     // Distance to enemy
    public double lateralVelocity;
    public double forwardWallDistance;
    public double reverseWallDistance;
    public double timeSinceDecel;
    public double guessFactor;  // Chosen guessFactor

    public double weight;

    public double[] getPoint ()
    {
        return new double[] {   distance / 800d,
                                lateralVelocity / 8d,
                                forwardWallDistance / Math.PI,
                                reverseWallDistance / Math.PI,
                                timeSinceDecel / 15d
                            };
    }
}
