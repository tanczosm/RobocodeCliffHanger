package ja;

import cg.RaikoGun;
import robocode.*;
import robocode.util.Utils;
import java.awt.geom.*;     // for Point2D's
import java.lang.*;         // for Double and Integer objects
import java.util.ArrayList; // for collection of waves
import java.awt.*;
import ja.dataStructures.trees.thirdGenKD.*;

public class CliffHanger extends AdvancedRobot {
    //public static int BINS = 47;
    //public static double _surfStats[] = new double[BINS]; // we'll use 47 bins
    public Point2D.Double _myLocation;     // our bot's location
    public Point2D.Double _enemyLocation;  // enemy bot's location

    public Point2D.Double _lastGoToPoint;
    public double direction = 1;

    public final boolean MOVEMENT_CHALLENGE = true;

    public static int MAX_SELECTED_SITUATIONS = 12;
    public static double KERNEL_DENSITY_BANDWIDTH = 14d;  // defaults to 6.0d

    public DistanceFunction distanceFunction = new ManhattanDistanceFunction();
    //public double[] SITUATION_WEIGHTS = {0.1};

    public final static int SITUATION_DIMENSIONS = 1;

    private ArrayList<Situation> _nodeQueue;
    public static KdTree<Situation> situations = new KdTree<Situation>(SITUATION_DIMENSIONS);

    public ArrayList _enemyWaves;
    public ArrayList _surfDirections;
    public ArrayList _surfAbsBearings;

    public ScannedRobotEvent _lastScan = null;
    public double _lastVelocity = 0;
    public int _lastVelocityChange = 0;

    public static RaikoGun _raikoGun;

    // We must keep track of the enemy's energy level to detect EnergyDrop,
// indicating a bullet is fired
    public static double _oppEnergy = 100.0;

    // This is a rectangle that represents an 800x600 battle field,
// used for a simple, iterative WallSmoothing method (by Kawigi).
// If you're not familiar with WallSmoothing, the wall stick indicates
// the amount of space we try to always have on either end of the tank
// (extending straight out the front or back) before touching a wall.
    public static Rectangle2D.Double _fieldRect
            = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
    public static double WALL_STICK = 160;

    public void run() {
        _enemyWaves = new ArrayList();
        _surfDirections = new ArrayList();
        _surfAbsBearings = new ArrayList();
        _nodeQueue = new ArrayList<Situation>(100);

        if (_raikoGun == null)
            _raikoGun = new RaikoGun(this);

        ((ManhattanDistanceFunction)distanceFunction).setSituationWeights(Situation.SITUATION_WEIGHTS);

        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        do {
            // basic mini-radar code
            if (!MOVEMENT_CHALLENGE)
                turnRadarRightRadians(Double.POSITIVE_INFINITY);
            else
                _raikoGun.run();
        } while (true);
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        _myLocation = new Point2D.Double(getX(), getY());

        if (_lastScan == null)
            _lastScan = e;


        double lateralVelocity = getVelocity()*Math.sin(e.getBearingRadians());
        double absBearing = e.getBearingRadians() + getHeadingRadians();


        double enemyHeading = getHeadingRadians() + e.getBearingRadians();
        double relativeHeading = e.getHeadingRadians() - enemyHeading;

        double velocity = e.getVelocity();
        double x = getX();
        double y = getY();
        double absVelocity = Math.abs(velocity);
        double enemyLateralVelocity = velocity * Math.sin(relativeHeading);
//        double forwardWallDistance = getWallTries(getHeadingRadians(), lateralVelocity > 0 ? 1 : -1, x, y, 15);
//        double reverseWallDistance = getWallTries(getHeadingRadians(), -lateralVelocity > 0 ? 1 : -1, x, y, 15);
        double forwardWallDistance = CTUtils.wallDistance(x, y, e.getDistance(), absBearing, 1);
        double reverseWallDistance = CTUtils.wallDistance(x, y, e.getDistance(), absBearing, -1);

        _lastVelocityChange++;
        if (Math.abs(_lastVelocity) - Math.abs(getVelocity()) >= 0.1) {
            _lastVelocityChange = 0;
        }
        _lastVelocity = getVelocity();

        setTurnRadarRightRadians(Utils.normalRelativeAngle(absBearing - getRadarHeadingRadians()) * 2);

        _surfDirections.add(0,
                new Integer((lateralVelocity >= 0) ? 1 : -1));
        _surfAbsBearings.add(0, new Double(absBearing + Math.PI));


        double bulletPower = _oppEnergy - e.getEnergy();
        if (bulletPower < 3.01 && bulletPower > 0.09
                && _surfDirections.size() > 2) {
            EnemyWave ew = new EnemyWave();
            ew.fireTime = getTime() - 1;
            ew.bulletVelocity = bulletVelocity(bulletPower);
            ew.distanceTraveled = bulletVelocity(bulletPower);
            ew.direction = ((Integer)_surfDirections.get(2)).intValue();
            ew.directAngle = ((Double)_surfAbsBearings.get(2)).doubleValue();
            ew.fireLocation = (Point2D.Double)_enemyLocation.clone(); // last tick
            ew.surfStats = new double[EnemyWave.BINS];
            ew.situation = new Situation();
            ew.situation.time = ew.fireTime;
            ew.situation.distance = e.getDistance();
            ew.situation.lateralVelocity = lateralVelocity;
            ew.situation.forwardWallDistance = forwardWallDistance;
            ew.situation.reverseWallDistance = reverseWallDistance;
            ew.situation.timeSinceDecel = Math.min(_lastVelocityChange, 15);
            ew.situation.weight = 1.0;

            System.out.println("Velocity: " + getVelocity() + ", Last velocity change: " + _lastVelocityChange);
            //System.out.println("FWD: " + forwardWallDistance/Math.PI*180 + ", RWD: " + reverseWallDistance/Math.PI*180);

            NearestNeighborIterator<Situation> similarSituationsIterator = situations.getNearestNeighborIterator(ew.situation.getPoint(), 6, distanceFunction);



            int count = 0;

            while (similarSituationsIterator.hasNext()) {
                Situation node = similarSituationsIterator.next();
                int index = getFactorIndex(node.guessFactor);
                int botWidth = getBotWidth(node.distance);

                double weight = node.weight;  // Number gets bigger as time passes
                //node.weight = Math.max(0, node.weight-0.05);

                for (int i = (int)Math.max(0, index-botWidth); i < (int)Math.min(index+botWidth, EnemyWave.BINS); i++)
                    ew.surfStats[i] += 1.0 * weight;
                count++;
            }
            System.out.println("Found " + count + " similar situations");
            if (count == 0)
            {
                int index = getFactorIndex(0);
                int botWidth = getBotWidth(ew.situation.distance);

                for (int i = (int)Math.max(0, index-botWidth); i < (int)Math.min(index+botWidth, EnemyWave.BINS); i++)
                    ew.surfStats[i] += 1.0;
            }

            _enemyWaves.add(ew);
        }

        _oppEnergy = e.getEnergy();

        // update after EnemyWave detection, because that needs the previous
        // enemy location as the source of the wave
        _enemyLocation = project(_myLocation, absBearing, e.getDistance());

        updateWaves();
        doSurfing();

        // gun code would go here...
        if (MOVEMENT_CHALLENGE)
            _raikoGun.onScannedRobot(e);

        _lastScan = e;
    }

    public int getBotWidth (double distance)
    {
        double botWidthAimAngle = CTUtils.botWidthAimAngle(distance - 34, 40.0);

        double gfSpan = 2.0 / (double)EnemyWave.BINS; // -1.0 to 1.0 is a 2.0 span over OUTPUT_LENGTH factors
        return (int)Math.max(Math.ceil(botWidthAimAngle / gfSpan), 1.0);

    }

    public Color getShade (double i)
    {
        i = 1 - CTUtils.clamp(i, 0.0, 1.0);

        // Heat map is red, orange, yellow, green, cyan, blue
        Color colors[] = {new Color(255, 20, 0),
                new Color(255, 112, 8),
                new Color(255, 245, 0),
                new Color(45, 255, 0),
                new Color(0, 228, 255),
                new Color(0, 74, 128)};

        return colors[(int)CTUtils.clamp(Math.round(i * colors.length), 0, colors.length - 1)];
    }

    public void drawWaves() {

        Graphics g = getGraphics();

        if (_lastGoToPoint != null)
        {
            g.setColor(Color.GREEN);
            g.drawOval((int)_lastGoToPoint.x-1, (int)_lastGoToPoint.y-1, 2, 2);
        }

        for (int i = 0; i < _enemyWaves.size(); i++) {

            EnemyWave ew = (EnemyWave)_enemyWaves.get(i);

            Point2D.Double center = ew.fireLocation;

            //int radius = (int)(w.distanceTraveled + w.bulletVelocity);

            double angleDivision = (CTUtils.maxEscapeAngle(ew.bulletVelocity) * 2.0 / (double)EnemyWave.BINS);

            int radius = (int)ew.getRadius(getTime());

            g.setColor(new Color(255, 188, 0));
            if (radius - 40 < center.distance(_myLocation))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            g.setColor(java.awt.Color.green);
            int cTime = (int) getTime();

            if (ew.surfStats != null) {

                double max = 0.01;
                for (int p = 0; p < ew.surfStats.length; p++)
                {
                    if (ew.surfStats[p] > max)
                        max = ew.surfStats[p];
                }

                for (int p = 0; p < ew.surfStats.length; p++) {

                    //float shade = (float) ew.waveGuessFactors[p];
                    //shade = (float) CTUtils.clamp(shade * 10, 0.2, 1.0);
                    //g.setColor(new Color(0, shade, 1, 1.0f));
                    int index = ew.direction >= 0 ? p : EnemyWave.BINS-1-p;

                    g.setColor(getShade(ew.surfStats[index]/max));

                    //System.out.print(shade + " ");
                    //System.out.println("DA: " + ew.directAngle + ", AD: " + angleDivision + ", MEA: " + ew.maxEscapeAngle);
                    double pangle = (ew.directAngle ) + (angleDivision * p) - (angleDivision * (EnemyWave.BINS / 2));

                    Point2D.Double p2 = CTUtils.project(center, pangle, radius);
                    Point2D.Double p3 = CTUtils.project(p2, pangle, (int) (ew.bulletVelocity));

//                g.drawOval((int) (p2.x - 1), (int) (p2.y - 1), 2, 2);
                    g.drawLine((int) p2.getX(), (int) p2.getY(), (int) p3.getX(), (int) p3.getY());
                }
            }
        }


    }

    public void updateWaves() {
        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave)_enemyWaves.get(x);


            drawWaves();

            ew.distanceTraveled = (getTime() - ew.fireTime) * ew.bulletVelocity;
            if (ew.distanceTraveled >
                    _myLocation.distance(ew.fireLocation) + 50) {
                _enemyWaves.remove(x);
                x--;
            }
        }
    }

    public EnemyWave getClosestSurfableWave() {
        double closestDistance = 50000; // I juse use some very big number here
        EnemyWave surfWave = null;

        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave)_enemyWaves.get(x);
            double distance = _myLocation.distance(ew.fireLocation)
                    - ew.distanceTraveled;

            if (distance > ew.bulletVelocity && distance < closestDistance) {
                surfWave = ew;
                closestDistance = distance;
            }
        }

        return surfWave;
    }

    public static double getFactor(EnemyWave ew, Point2D.Double targetLocation) {
        double offsetAngle = (absoluteBearing(ew.fireLocation, targetLocation)
                - ew.directAngle);
        double factor = Utils.normalRelativeAngle(offsetAngle)
                / maxEscapeAngle(ew.bulletVelocity) * ew.direction;

        return factor;
    }

    // Given the EnemyWave that the bullet was on, and the point where we
// were hit, calculate the index into our stat array for that factor.
    public static int getFactorIndex(double factor) {

        return (int)limit(0,
                (factor * ((EnemyWave.BINS - 1) / 2)) + ((EnemyWave.BINS - 1) / 2),
                EnemyWave.BINS - 1);
    }


    // Given the EnemyWave that the bullet was on, and the point where we
// were hit, update our stat array to reflect the danger in that area.
    public void logHit(EnemyWave ew, Point2D.Double targetLocation) {


        ew.situation.guessFactor = getFactor(ew, targetLocation);

        situations.addPoint(ew.situation.getPoint(), ew.situation);

        /*
        for (int x = 0; x < BINS; x++) {
            // for the spot bin that we were hit on, add 1;
            // for the bins next to it, add 1 / 2;
            // the next one, add 1 / 5; and so on...
            _surfStats[x] += 1.0 / (Math.pow(index - x, 2) + 1);
        }
        */
    }

    @Override
    public void onBulletHitBullet(BulletHitBulletEvent e)
    {
        handleBulletEvent(e.getHitBullet(), true);
    }


    public void onHitByBullet(HitByBulletEvent e) {
        handleBulletEvent(e.getBullet(), false);
    }

    public void handleBulletEvent (Bullet bullet, boolean bulletHitBullet)
    {
        // If the _enemyWaves collection is empty, we must have missed the
        // detection of this wave somehow.
        if (!_enemyWaves.isEmpty()) {
            Point2D.Double hitBulletLocation = new Point2D.Double(
                    bullet.getX(), bullet.getY());
            EnemyWave hitWave = null;

            // look through the EnemyWaves, and find one that could've hit us.
            for (int x = 0; x < _enemyWaves.size(); x++) {
                EnemyWave ew = (EnemyWave)_enemyWaves.get(x);

                if (bulletHitBullet) {
                    if (Math.abs(ew.distanceTraveled -
                            _myLocation.distance(ew.fireLocation)) < 50
                            && Math.abs(bulletVelocity(bullet.getPower())
                            - ew.bulletVelocity) < 0.001) {
                        hitWave = ew;
                        break;
                    }
                }
                else
                {
                    Point2D.Double pt = new Point2D.Double(bullet.getX(), bullet.getY());

                    if (Math.abs(ew.distanceTraveled -
                            pt.distance(ew.fireLocation)) < 50
                            && Math.abs(bulletVelocity(bullet.getPower())
                            - ew.bulletVelocity) < 0.001) {
                        hitWave = ew;
                        break;
                    }

                }
            }

            if (hitWave != null) {
                logHit(hitWave, hitBulletLocation);



                // We can remove this wave now, of course.
                _enemyWaves.remove(_enemyWaves.lastIndexOf(hitWave));
            }
        }
    }

    // CREDIT: mini sized predictor from Apollon, by rozu
// http://robowiki.net?Apollon
    public ArrayList predictPositions(EnemyWave surfWave, int direction) {
        Point2D.Double predictedPosition = (Point2D.Double)_myLocation.clone();
        double predictedVelocity = getVelocity();
        double predictedHeading = getHeadingRadians();
        double maxTurning, moveAngle, moveDir;
        ArrayList traveledPoints = new ArrayList();

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;

        do {
            double distance = predictedPosition.distance(surfWave.fireLocation);
            double offset = Math.PI/2 - 1 + distance/400;

            moveAngle =
                    wallSmoothing(predictedPosition, absoluteBearing(surfWave.fireLocation,
                            predictedPosition) + (direction * (offset)), direction)
                            - predictedHeading;
            moveDir = 1;

            if(Math.cos(moveAngle) < 0) {
                moveAngle += Math.PI;
                moveDir = -1;
            }

            moveAngle = Utils.normalRelativeAngle(moveAngle);

            // maxTurning is built in like this, you can't turn more then this in one tick
            maxTurning = Math.PI/720d*(40d - 3d*Math.abs(predictedVelocity));
            predictedHeading = Utils.normalRelativeAngle(predictedHeading
                    + limit(-maxTurning, moveAngle, maxTurning));

            // this one is nice ;). if predictedVelocity and moveDir have
            // different signs you want to breack down
            // otherwise you want to accelerate (look at the factor "2")
            predictedVelocity += (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir);
            predictedVelocity = limit(-8, predictedVelocity, 8);

            // calculate the new predicted position
            predictedPosition = project(predictedPosition, predictedHeading, predictedVelocity);

            //add this point the our prediction
            traveledPoints.add(predictedPosition);

            counter++;

            if (predictedPosition.distance(surfWave.fireLocation) - 20 <
                    surfWave.distanceTraveled + (counter * surfWave.bulletVelocity)
                   + surfWave.bulletVelocity
                    ) {
                intercepted = true;
            }
        } while(!intercepted && counter < 500);

        //we can't get the the last point, because we need to slow down
        if(traveledPoints.size() > 1)
            traveledPoints.remove(traveledPoints.size() - 1);

        return traveledPoints;
    }

    public double checkDanger(EnemyWave surfWave, Point2D.Double position) {
        int index = getFactorIndex(getFactor(surfWave, position));
        double distance = position.distance(surfWave.fireLocation);
        return surfWave.surfStats[index]/distance;
    }

    public double checkDangerSpan(EnemyWave surfWave,  ArrayList<Point2D.Double> forwardPoints, int center, int botWidth)
    {
        /*
        Point2D.Double position = (Point2D.Double)(forwardPoints.get(center));
        double distance = position.distance(surfWave.fireLocation);
        double dangerSum = 0;
        double count = 0;


        for (Point2D.Double pt : forwardPoints)
        {
            System.out.println("dist: " + pt.distance(position));
            if (pt.distance(position) <= botWidth)
            {
                int idx = getFactorIndex(getFactor(surfWave, pt));
                dangerSum += surfWave.surfStats[idx];
                count++;
            }
        }
        System.out.println("danger check with botWidth: " + botWidth + ", dangerSum: " + dangerSum + ", count: " + count);

        return dangerSum; // / count / distance;
        */
        Point2D.Double position = (Point2D.Double)(forwardPoints.get(center));
        int index = getFactorIndex(getFactor(surfWave, position));
        double distance = position.distance(surfWave.fireLocation);

        double dangerSum = 0;
        double count = 0;
        for (int i = (int)Math.max(0, index-botWidth); i < (int)Math.min(index+botWidth, EnemyWave.BINS); i++) {
            dangerSum += surfWave.surfStats[index];
            count++;
        }
        System.out.println("danger check with botWidth: " + botWidth + ", dangerSum: " + dangerSum + ", count: " + count);
        return dangerSum;// / distance;

    }

    public Point2D.Double getBestPoint(EnemyWave surfWave){
        if(surfWave.safePoints == null){
            ArrayList<Point2D.Double> forwardPoints = predictPositions(surfWave, 1);
            ArrayList<Point2D.Double> reversePoints = predictPositions(surfWave, -1);
            int FminDangerIndex = 0;
            int RminDangerIndex = 0;
            int botWidth = 0;
            double FminDanger = Double.POSITIVE_INFINITY;
            double RminDanger = Double.POSITIVE_INFINITY;
            double FDangerSum = 0;
            double RDangerSum = 0;
            System.out.println("Checking forward points..");
            for(int i = 0, k = forwardPoints.size(); i < k; i++){
                Point2D.Double center = (Point2D.Double)(forwardPoints.get(i));
                botWidth = getBotWidth(center.distance(surfWave.fireLocation));
                double thisDanger = checkDangerSpan(surfWave, forwardPoints, i, botWidth ); // checkDanger(surfWave, (Point2D.Double)(forwardPoints.get(i)));

                FDangerSum += thisDanger;

                if(thisDanger <= FminDanger){

                    FminDangerIndex = i;
                    FminDanger = thisDanger;
                }
            }
            System.out.println("Checking reverse points..");

            for(int i = 0, k = reversePoints.size(); i < k; i++){
                Point2D.Double center = (Point2D.Double)(reversePoints.get(i));
                botWidth = getBotWidth(center.distance(surfWave.fireLocation));

                double thisDanger = checkDangerSpan(surfWave, reversePoints, i, botWidth ); // checkDanger(surfWave, (Point2D.Double)(reversePoints.get(i)));

                RDangerSum += thisDanger;
                if(thisDanger <= RminDanger){

                    RminDangerIndex = i;
                    RminDanger = thisDanger;
                }
            }
            ArrayList bestPoints;
            int minDangerIndex;

            System.out.println("FminDanger: " + FminDanger + ", RminDanger: " + RminDanger);



            if(FminDanger < RminDanger ){
                bestPoints = forwardPoints;
                minDangerIndex = FminDangerIndex;
            }
            else if (FminDanger > RminDanger ) {
                bestPoints = reversePoints;
                minDangerIndex = RminDangerIndex;
            }
            else
            {
                // All this considered equal, go to the less dangerous side
                if (FDangerSum < RDangerSum)
                {
                    bestPoints = forwardPoints;
                    minDangerIndex = FminDangerIndex;
                }
                else
                {
                    bestPoints = reversePoints;
                    minDangerIndex = RminDangerIndex;
                }

            }

            Point2D.Double bestPoint = (Point2D.Double)bestPoints.get(minDangerIndex);

            while(bestPoints.indexOf(bestPoint) != -1)
                bestPoints.remove(bestPoints.size() - 1);
            bestPoints.add(bestPoint);

            surfWave.safePoints = bestPoints;

            //debugging - so that we should always be on top of the last point
            bestPoints.add(0,new Point2D.Double(getX(), getY()));

        }
        else
        if(surfWave.safePoints.size() > 1)
            surfWave.safePoints.remove(0);

        Graphics g = getGraphics();

        if(surfWave.safePoints.size() >= 1){
            for(int i = 0,k=surfWave.safePoints.size(); i < k; i++){
                Point2D.Double goToPoint = (Point2D.Double)surfWave.safePoints.get(i);
                g.drawOval((int)goToPoint.x-1, (int)goToPoint.y-1, 2,2);
                if(goToPoint.distanceSq(_myLocation) > 20*20*1.1)
                    //if it's not 20 units away we won't reach max velocity
                    return goToPoint;
            }
            //if we don't find a point 20 units away, return the end point
            return (Point2D.Double)surfWave.safePoints.get(surfWave.safePoints.size() - 1);


        }

        return null;
    }

    public void doSurfing() {
        EnemyWave surfWave = getClosestSurfableWave();
        double distance = _enemyLocation.distance(_myLocation);
        if (surfWave == null || distance < 50) {
            //do 'away' movement  best distance of 400 - modified from RaikoNano
            double absBearing = absoluteBearing(_myLocation, _enemyLocation);
            double headingRadians = getHeadingRadians();
            double stick = 160;//Math.min(160,distance);
            double  v2, offset = Math.PI/2 + 1 - distance/400;

            while(!_fieldRect.
                    contains(project(_myLocation,v2 = absBearing + direction*(offset -= 0.02), stick)

                            // 	getX() + stick * Math.sin(v2 = absBearing + direction * (offset -= .02)), getY() + stick * Math.cos(v2)
                    ));


            if( offset < Math.PI/3 )
                direction = -direction;
            setAhead(50*Math.cos(v2 - headingRadians));
            setTurnRightRadians(Math.tan(v2 - headingRadians));

        }
        else
            goTo(getBestPoint(surfWave));
    }
    private void goTo(Point2D.Double destination) {
        if(destination == null){
            if(_lastGoToPoint != null)
                destination = _lastGoToPoint;
            else
                return;
        }

        _lastGoToPoint = destination;
        Point2D.Double location = new Point2D.Double(getX(), getY());
        double distance = location.distance(destination);
        double angle = Utils.normalRelativeAngle(absoluteBearing(location, destination) - getHeadingRadians());
        if (Math.abs(angle) > Math.PI/2) {
            distance = -distance;
            if (angle > 0) {
                angle -= Math.PI;
            }
            else {
                angle += Math.PI;
            }
        }

        //this is hacked so that the bot doesn't turn once we get to our destination
        setTurnRightRadians(angle*Math.signum(Math.abs((int)distance)));

        setAhead(distance);
    }

    // This can be defined as an inner class if you want.


    // CREDIT: Iterative WallSmoothing by Kawigi
//   - return absolute angle to move at after account for WallSmoothing
// robowiki.net?WallSmoothing
    public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
        while (!_fieldRect.contains(project(botLocation, angle, 160))) {
            angle += orientation*0.05;
        }
        return angle;
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

    public static double limit(double min, double value, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double bulletVelocity(double power) {
        return (20D - (3D*power));
    }

    public static double maxEscapeAngle(double velocity) {
        return Math.asin(8.0/velocity);
    }

    public static void setBackAsFront(AdvancedRobot robot, double goAngle) {
        double angle =
                Utils.normalRelativeAngle(goAngle - robot.getHeadingRadians());
        if (Math.abs(angle) > (Math.PI/2)) {
            if (angle < 0) {
                robot.setTurnRightRadians(Math.PI + angle);
            }
            else {
                robot.setTurnLeftRadians(Math.PI - angle);
            }
            robot.setBack(100);
        }
        else {
            if (angle < 0) {
                robot.setTurnLeftRadians(-1*angle);
            }
            else {
                robot.setTurnRightRadians(angle);
            }
            robot.setAhead(100);
        }
    }

    public double getWallTries(double heading, double dir, double x, double y, double distance){

        Graphics2D g = getGraphics();
        double wallIncrement = 0.0407d * dir;
        double eHeading = heading;
        double nextX = 0;
        double nextY = 0;
        double wallTries = -1;
        System.out.println("x: " + x + " y: " + y + " heading: " + heading);
        do{
            eHeading += wallIncrement;
            nextX = x + Math.sin(eHeading) * distance;
            nextY = y + Math.cos(eHeading) * distance;
            g.drawOval((int)nextX, (int)nextY, 2, 2);
            wallTries++;
        }while(_fieldRect.contains(nextX, nextY) && wallTries < 20);

        return wallTries;
    }


    /*
    public void onPaint(java.awt.Graphics2D g) {
        g.setColor(Color.red);
        for(int i = 0; i < _enemyWaves.size(); i++){
            EnemyWave w = (EnemyWave)(_enemyWaves.get(i));
            Point2D.Double center = w.fireLocation;

            //int radius = (int)(w.distanceTraveled + w.bulletVelocity);
            //hack to make waves line up visually, due to execution sequence in robocode engine
            //use this only if you advance waves in the event handlers (eg. in onScannedRobot())
            //NB! above hack is now only necessary for robocode versions before 1.4.2
            //otherwise use:
            int radius = (int)w.distanceTraveled;

            if(radius - 40 < center.distance(_myLocation))
                g.drawOval((int)(center.x - radius ), (int)(center.y - radius), radius*2, radius*2);
        }
    }
    */
}