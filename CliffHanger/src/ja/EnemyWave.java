package ja;

import java.awt.geom.Point2D;
import java.util.ArrayList;

class EnemyWave {

    public final static int BINS = 47;

    Point2D.Double fireLocation;
    long fireTime;
    double bulletVelocity, directAngle, distanceTraveled;
    int direction;
    ArrayList safePoints;
    double[] surfStats = new double[BINS];

    Situation situation; // We will save this once we are done with the wave

    public EnemyWave() { }

    public double getRadius(long time) {
        return bulletVelocity*(time - fireTime);
    }

}