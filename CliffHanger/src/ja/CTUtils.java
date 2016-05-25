package ja;

import robocode.AdvancedRobot;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

/**
 * Created by tanczosm on 5/29/2015.
 */
public class CTUtils {

    public static MovSim moveSimulator;

    public static final double HALF_PI = Math.PI / 2;
    public static final double WALKING_STICK = 120;
    public static final double WALL_MARGIN = 18;
    public static final double S = WALL_MARGIN;
    public static final double W = WALL_MARGIN;
    public static final double N = 600 - WALL_MARGIN;
    public static final double E = 800 - WALL_MARGIN;

    // eDist  = the distance from you to the enemy
    // eAngle = the absolute angle from you to the enemy
    // oDir   =  1 for the clockwise orbit distance
    //          -1 for the counter-clockwise orbit distance
    // returns: the positive orbital distance (in radians) the enemy can travel
    //          before hitting a wall (possibly infinity).
    public static double wallDistance(double playerX, double playerY, double eDist, double eAngle, int oDir) {
        return Math.min(Math.min(Math.min(
                                distanceWest(N - playerY, eDist, eAngle - HALF_PI, oDir),
                                distanceWest(E - playerX, eDist, eAngle + Math.PI, oDir)),
                        distanceWest(playerY - S, eDist, eAngle + HALF_PI, oDir)),
                distanceWest(playerX - W, eDist, eAngle, oDir));
    }

    public static double distanceWest(double toWall, double eDist, double eAngle, int oDir) {
        if (eDist <= toWall) {
            return Double.POSITIVE_INFINITY;
        }
        double wallAngle = Math.acos(-oDir * toWall / eDist) + oDir * HALF_PI;
        return Utils.normalAbsoluteAngle(oDir * (wallAngle - eAngle));
    }


    /**
     * Returns an array of vectors where the rectangle intersect a circle
     * at c with radius r.
     */
    public static final Point2D.Double[] intersectRectCircle(Rectangle2D.Double rect, Point2D.Double c, double r) {
        double[] pnts = intersectRectCircle(rect.getMinX(),rect.getMinY(),
                rect.getWidth(),rect.getHeight(), c.x, c.y, r);
        Point2D.Double[] output = new Point2D.Double[pnts.length/2];
        for(int i = 0; i<output.length; ++i) {
            output[i] = new Point2D.Double(pnts[i*2],pnts[i*2+1]);
        }
        return output;
    }

    public static final double[][] intersectRectCircleD(Rectangle2D.Double rect, Point2D.Double c, double r) {
        double[] pnts = intersectRectCircle(rect.getMinX(),rect.getMinY(),
                rect.getWidth(),rect.getHeight(), c.x, c.y, r);
        double[][] output = new double[pnts.length/2][2];
        for(int i = 0; i<output.length; ++i) {
            output[i] = new double[]{pnts[i*2],pnts[i*2+1]};
        }
        return output;
    }

    public static final double[] intersectRectCircle(
            double rx, double ry, double rw, double rh,
            double cx, double cy, double r) {
        double mx = rx+rw;
        double my = ry+rh;

        //every line can intersect twice, meaning 4 points at most per line
        double[] intersect = new double[16];
        int n = 0;

        double[] in = intersectSegCircle(cx,cy,r,rx,ry,mx,ry); //top
		/*
		 * for(int i=0;i!=in.length;++i)
		 *     intersect[n++] = in[i];
		 *
		 * Equivalent to below, just the hardcoded ifs are faster
		 */
        if(in.length == 2) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
        } else if(in.length == 4) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
            intersect[n++] = in[2];
            intersect[n++] = in[3];
        }

        in = intersectSegCircle(cx,cy,r,rx,my,mx,my); //bottom
        if(in.length == 2) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
        } else if(in.length == 4) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
            intersect[n++] = in[2];
            intersect[n++] = in[3];
        }

        in = intersectSegCircle(cx,cy,r,rx,ry,rx,my); //left
        if(in.length == 2) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
        } else if(in.length == 4) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
            intersect[n++] = in[2];
            intersect[n++] = in[3];
        }

        in = intersectSegCircle(cx,cy,r,mx,ry,mx,my); //right
        if(in.length == 2) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
        } else if(in.length == 4) {
            intersect[n++] = in[0];
            intersect[n++] = in[1];
            intersect[n++] = in[2];
            intersect[n++] = in[3];
        }

        double[] output = new double[n];
        for(int i=0;i!=n;++i)
            output[i] = intersect[i];

        return output;
    }

    public static final double[] intersectSegCircle(double cx, double cy, double r,
                                                    double lax, double lay, double lbx, double lby) {

        double diffx = cx - lax;
        double diffy = cy - lay;

        double dirx = lbx-lax;
        double diry = lby-lay;
        double l = Math.sqrt(dirx * dirx + diry * diry);

        dirx /= l;
        diry /= l;

        double a0 = diffx*diffx+diffy*diffy - r*r;
        double a1 = diffx*dirx+diffy*diry;

        double discr = a1 * a1 - a0;

        if (discr > 0) {
			/* The circle and line meet at two places */
            double lengthSq = (lbx-lax)*(lbx-lax)+(lby-lay)*(lby-lay);

            discr = Math.sqrt(discr);
            double m1 = a1 - discr;
            double m2 = a1 + discr;

            if(m1 > 0 && m1*m1 < lengthSq && m2 > 0 && m2*m2 < lengthSq) {
                return new double[] {
                        lax + m1 * dirx, lay + m1 * diry,
                        lax + m2 * dirx, lay + m2 * diry
                };
            } else if (m1 > 0 && m1*m1 < lengthSq) {
                return new double[] {
                        lax + m1 * dirx, lay + m1 * diry
                };
            } else if (m2 > 0 && m2*m2 < lengthSq) {
                return new double[] {
                        lax + m2 * dirx, lay + m2 * diry
                };
            }
        } else if (discr == 0) {
            double lengthSq = (lbx-lax)*(lbx-lax)+(lby-lay)*(lby-lay);
			/* We have ourselves a tangent */
            if (a1 > 0 && a1*a1 < lengthSq) {
                return new double[] {
                        lax+a1*dirx, lay+a1*diry
                };
            }
        }

        return new double[0];
    }


    // CREDIT: from CassiusClay, by PEZ
    //   - returns point length away from sourceLocation, at angle
    // robowiki.net?CassiusClay
    public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
        return new Point2D.Double(sourceLocation.x + Math.sin(angle) * length,
                sourceLocation.y + Math.cos(angle) * length);
    }

    // got this from RaikoMicro, by Jamougha, but I think it's used by many authors
    //  - returns the absolute angle (in radians) from source to target points
    public static double absoluteBearing(Point2D.Double source, Point2D.Double target) {
        return Math.atan2(target.x - source.x, target.y - source.y);
    }


    public static int bulletTicks(double distance, double power) {
        return (int) Math.ceil(distance / (20 - (3 * power)));
    }

    public static double bulletDamage(double power) {
        return (power * 4) + (2 * Math.max(power - 1, 0));
    }

    public static double bulletLifeGain(double power) {
        return (3 * power);
    }

    public static double gunHeat(double power) {
        return 1 + (power / 5);
    }

    public static double gunHeatTicks(double power, double coolingRate) {
        return Math.ceil((1 + (power / 5)) / coolingRate);
    }

    public static double botWidthAimAngle(double distance, double width) {
        return Math.abs(width / distance);
    }
    public static double botWidthAimAngle(double distance) {
        return Math.abs(18.0 / distance);
    }

    public static Point2D.Double nextLocation(AdvancedRobot robot) {
        if (moveSimulator == null) {
            moveSimulator = new MovSim();
        }

        MovSimStat[] next = moveSimulator.futurePos(1, robot);

        return new Point2D.Double(next[0].x, next[0].y);
    }

    public static MovSimStat nextLocationFull(AdvancedRobot robot) {
        if (moveSimulator == null) {
            moveSimulator = new MovSim();
        }

        MovSimStat[] next = moveSimulator.futurePos(1, robot);

        return next[0];
    }

    /*
        public static Point2D.Double nextLocation(BotScan scan) {
        return new Point2D.Double(scan.getLocation().x
            + (Math.sin(scan.getHeadingRadians())*scan.getVelocity()),
            scan.getLocation().y
            + (Math.cos(scan.getHeadingRadians())*scan.getVelocity()));
    }
     */

    public static MovSim getMovSim() {
        if (moveSimulator == null) {
            moveSimulator = new MovSim();
        }

        return moveSimulator;
    }

    public static int getOrientation(double targetHeadingRadians,
                                     double targetVelocity, double absBearingRadians) {

        double relativeHeadingRadians = Utils.normalRelativeAngle(
                targetHeadingRadians - absBearingRadians +
                        (targetVelocity < 0 ? Math.PI : 0));

        return sign(relativeHeadingRadians);
    }

    public static RobotState nextPerpendicularLocation(
            Point2D.Double targetLocation, double absBearingRadians,
            double enemyVelocity, double enemyHeadingRadians,
            boolean clockwise, long currentTime, boolean ignoreWallHits) {

        int purelyPerpendicularOffset = 0;

        return nextPerpendicularLocation(targetLocation, absBearingRadians,
                enemyVelocity, enemyHeadingRadians, purelyPerpendicularOffset,
                clockwise, currentTime, ignoreWallHits);
    }

    public static RobotState nextPerpendicularLocation(
            Point2D.Double targetLocation, double absBearingRadians,
            double enemyVelocity, double enemyHeadingRadians, double attackAngle,
            boolean clockwise, long currentTime, boolean ignoreWallHits) {

        return nextPerpendicularWallSmoothedLocation(targetLocation,
                absBearingRadians, enemyVelocity, 8.0, enemyHeadingRadians,
                attackAngle, clockwise, currentTime, null, 0, 0, 0,
                ignoreWallHits);
    }

    public static RobotState nextPerpendicularWallSmoothedLocation(
            Point2D.Double targetLocation, double absBearingRadians,
            double enemyVelocity, double maxVelocity, double enemyHeadingRadians,
            double attackAngle, boolean clockwise, long currentTime,
            Rectangle2D.Double battleField, double bfWidth, double bfHeight,
            double wallStick, boolean ignoreWallHits) {

        int orientation;
        if (clockwise) {
            orientation = 1;
        } else {
            orientation = -1;
        }

        double goAngleRadians = Utils.normalRelativeAngle(
                absBearingRadians + (orientation * ((Math.PI / 2) + attackAngle)));

        boolean isSmoothing = false;

        if (wallStick != 0 && battleField != null) {
            double smoothedAngle = CTUtils.wallSmoothing(
                    battleField, bfWidth, bfHeight, targetLocation,
                    goAngleRadians, orientation, wallStick);

            if (CTUtils.round(smoothedAngle, 4)
                    != CTUtils.round(goAngleRadians, 4)) {
                isSmoothing = true;
            }

            goAngleRadians = smoothedAngle;
        }

        return nextLocation(targetLocation, enemyVelocity, maxVelocity,
                enemyHeadingRadians, goAngleRadians, currentTime,
                isSmoothing, ignoreWallHits, bfWidth, bfHeight);
    }

    public static RobotState nextLocation(
            Point2D.Double targetLocation, double enemyVelocity, double maxVelocity,
            double enemyHeadingRadians, double goAngleRadians, long currentTime,
            boolean isSmoothing, boolean ignoreWallHits, double bfWidth, double bfHeight) {

        MovSim movSim = CTUtils.getMovSim();

        double futureTurn = Utils.normalRelativeAngle(
                goAngleRadians - enemyHeadingRadians);
        double futureDistance;

        if (Math.abs(futureTurn) > (Math.PI / 2)) {
            if (futureTurn < 0) {
                futureTurn = Math.PI + futureTurn;
            } else {
                futureTurn = -1 * (Math.PI - futureTurn);
            }

            futureDistance = -1000;
        } else {
            futureDistance = 1000;
        }

        int extraWallSize = 0;

        if (ignoreWallHits) {
            extraWallSize = 50000;
        }

        MovSimStat[] futureMoves = movSim.futurePos(
                1, extraWallSize + targetLocation.x,
                extraWallSize + targetLocation.y, enemyVelocity, maxVelocity,
                enemyHeadingRadians, futureDistance,
                futureTurn, 10.0, extraWallSize * 2 + bfWidth,
                extraWallSize * 2 + bfHeight);

        return new RobotState(
                new Point2D.Double(CTUtils.round(futureMoves[0].x - extraWallSize, 3),
                        CTUtils.round(futureMoves[0].y - extraWallSize, 3)), futureMoves[0].h,
                futureMoves[0].v, currentTime + 1, isSmoothing);
    }

    public static double round(double d, int i) {
        long powerTen = 1;

        for (int x = 0; x < i; x++) {
            powerTen *= 10;
        }

        return ((double) Math.round(d * powerTen)) / powerTen;
    }

    public static double cornerDistance (Point2D.Double location, double bfWidth,
                                         double bfHeight)
    {
        double tr = location.distance(bfWidth, bfHeight);
        double tl = location.distance(0, bfHeight);
        double br = location.distance(bfWidth, 0);
        double bl = location.distance(0, 0);

        return Math.min(Math.min(tr,tl), Math.min(br,bl));
    }


    /**
     * wallSmoothing: do some Voodoo and wall smooth in a very efficiently.
     * - ...in terms of CPU cycles, not amount of code.
     * - used to be iterative, which was a lot simpler and more readable,
     * but far too slow with how much it was called during precise
     * prediction.
     */
    public static double wallSmoothing(Rectangle2D.Double field, double bfWidth,
                                       double bfHeight, Point2D.Double startLocation, double startAngleRadians,
                                       int orientation, double wallStick) {

/*
        double angle = startAngle;
        _lastWallSmoothAway = false;
        while (!field.contains(x + Math.sin(Math.toRadians(angle))*WALL_STICK,
            y+Math.cos(Math.toRadians(angle))*WALL_STICK)) {
            angle += orientation*smoothNormal*7.0;
            if (smoothNormal == -1) { _lastWallSmoothAway = true; }
        }
        return angle;
*/
        // Trying to do almost exactly the equivalent of the above in more
        // code but less CPU time. The above needs a low increment to work
        // perfectly smoothly, which results in very slow execution.
        //
        // NOTE: The two algorithms can give slightly different results,
        //       but that is mainly because the iterative one never tests a
        //       very specific angle in a corner that would turn up "in bounds";
        //       if it increased the angle var by (1/INFINITY), they'd be the
        //       same (as far as I can tell.)

        double angle = startAngleRadians;
        double wallDistanceX = Math.min(startLocation.x - 18,
                bfWidth - startLocation.x - 18);
        double wallDistanceY = Math.min(startLocation.y - 18,
                bfHeight - startLocation.y - 18);

        if (wallDistanceX > wallStick && wallDistanceY > wallStick) {
            return startAngleRadians;
        }

        double testX = startLocation.x + (Math.sin(angle) * wallStick);
        double testY = startLocation.y + (Math.cos(angle) * wallStick);
        double testDistanceX = Math.min(testX - 18, bfWidth - testX - 18);
        double testDistanceY = Math.min(testY - 18, bfHeight - testY - 18);

        double adjacent = 0;
        int g = 0;

        while (!field.contains(testX, testY) && g++ < 25) {
            if (angle < 0) {
                angle += (2 * Math.PI);
            }
            if (testDistanceY < 0 && testDistanceY < testDistanceX) {
                // wall smooth North or South wall
                angle = ((int) ((angle + (Math.PI / 2)) / Math.PI)) * Math.PI;
                adjacent = Math.abs(wallDistanceY);
            } else if (testDistanceX < 0 && testDistanceX <= testDistanceY) {
                // wall smooth East or West wall
                angle = (((int) (angle / Math.PI)) * Math.PI) + (Math.PI / 2);
                adjacent = Math.abs(wallDistanceX);
            }

            angle += orientation *
                    (Math.abs(Math.acos(adjacent / wallStick)) + 0.0005);

            testX = startLocation.x + (Math.sin(angle) * wallStick);
            testY = startLocation.y + (Math.cos(angle) * wallStick);
            testDistanceX = Math.min(testX - 18, bfWidth - testX - 18);
            testDistanceY = Math.min(testY - 18, bfHeight - testY - 18);
        }

        return angle;
    }

    //
    private static double getNewVelocity(double velocity, double distance) {
        final double goalVel = Math.min(getMaxVelocity(distance), 8);

        if (velocity >= 0)
            return limit(velocity - 2,
                    goalVel, velocity + 1);

        return limit(velocity - 1,
                goalVel, velocity + maxDecel(-velocity));
    }

    // Returns the peak velocity for distance
    final static double getMaxVelocity(double distance) {
        final double decelTime = Math.max(1, Math.ceil(
                //sum of 0... decelTime, solving for decelTime
                //using quadratic formula, then simplified a lot
                Math.sqrt(distance + 1) - 0.5));

        final double decelDist = (decelTime) * (decelTime - 1);
        // sum of 0..(decelTime-1)
        // * Rules.DECELERATION*0.5;c

        return ((decelTime - 1) * 2) + ((distance - decelDist) / decelTime);
    }

    // Returns the peak amount of deceleration that can be performed if traveling at speed
    private static final double maxDecel(double speed) {
        return limit(1, speed * 0.5 + 1, 2);
    }

    public static int sign(double p) {
        return p < 0 ? -1 : 1;
    }

    public static double limit(double min, double value, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double bulletVelocity(double power) {
        return (20D - (3D * power));
    }

    public static double maxEscapeAngle(double velocity) {
        return Math.asin(8.0 / velocity);
    }

    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public static void setBackAsFront(AdvancedRobot robot, double goAngle) {
        double angle =
                Utils.normalRelativeAngle(goAngle - robot.getHeadingRadians());
        if (Math.abs(angle) > (Math.PI / 2)) {
            if (angle < 0) {
                robot.setTurnRightRadians(Math.PI + angle);
            } else {
                robot.setTurnLeftRadians(Math.PI - angle);
            }
            robot.setBack(Double.POSITIVE_INFINITY);
        } else {
            if (angle < 0) {
                robot.setTurnLeftRadians(-1 * angle);
            } else {
                robot.setTurnRightRadians(angle);
            }
            robot.setAhead(Double.POSITIVE_INFINITY); //Math.cos(angle) * Double.POSITIVE_INFINITY);
        }
    }

}