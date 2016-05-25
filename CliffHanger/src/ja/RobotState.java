package ja;


import java.awt.geom.Point2D;

public class RobotState implements Comparable<RobotState> {
    public Point2D.Double location;
    public double heading;
    public double velocity;
    public double maxVelocity = 8;
    public int direction;
    public double angleToTurn;
    public double distanceRemaining;
    public long time;
    public boolean smoothing;
    public double danger;
    public int tickDistance;
    public double dangerDensity; // When combined with adjacent states, how dangerous is this spot?
    public boolean reachable; // True if reachable WITHOUT NEEDING PRECISE CHECKING

    public RobotState(Point2D.Double botLocation, double botHeadingRadians,
                      double botVelocity) {

        location = botLocation;
        heading = botHeadingRadians;
        velocity = botVelocity;
        direction = 1;
        smoothing = false;
        danger = Double.MAX_VALUE;
        distanceRemaining = 0;
        reachable = false;
    }

    public RobotState(Point2D.Double botLocation, double botHeadingRadians,
                      double botVelocity, long currentTime) {

        this(botLocation, botHeadingRadians, botVelocity);

        time = currentTime;
    }

    public RobotState(Point2D.Double botLocation, double botHeadingRadians,
                      double botVelocity, long currentTime, boolean smooth) {

        this(botLocation, botHeadingRadians, botVelocity, currentTime);

        smoothing = smooth;
    }

    public RobotState(Point2D.Double botLocation, double botHeadingRadians,
                      double botVelocity, long currentTime, double distanceRemain, double turnAngle) {

        this(botLocation, botHeadingRadians, botVelocity, currentTime);

        this.distanceRemaining = distanceRemain;
        this.angleToTurn = turnAngle;
    }

    public Object clone() {
        RobotState newState = new RobotState((Point2D.Double) location.clone(), heading,
                velocity, time, distanceRemaining, angleToTurn);

        newState.reachable = this.reachable;
        newState.danger = this.danger;
        newState.direction = this.direction;

        return newState;
    }

    public int compareTo(RobotState other)
    {
        if (danger < other.danger)
            return -1;
        else if (danger == other.danger)
            return 0;

        return 1;
    }

}