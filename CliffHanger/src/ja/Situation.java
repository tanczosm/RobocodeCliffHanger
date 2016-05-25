package ja;

/**
 * Created by eahscs on 5/25/2016.
 */
public class Situation {

    public int time;
    public double distance;     // Distance to enemy
    public double guessFactor;  // Chosen guessFactor

    public double[] getPoint ()
    {
        return new double[] { distance / 800d };
    }
}
