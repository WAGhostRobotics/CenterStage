package org.firstinspires.ftc.teamcode.library.autoDrive.math;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;

public class Bezier {

    Point[] waypoints;

    double pow;
    double pow2;
    double pow3;
    double pow4;

    public double heading;
    public int pointCount;
    private boolean headingFollower;

    private Point[] curvePoints;
    private Point[] curveDerivatives;
    private double[] curveHeadings;
    private final double defaultIncrement = MotionPlanner.tIncrement;
    public int estimatedStopping;
    private final double endTrajThreshold = 10;
    private double approxLength;
    static double tIncrement;

    public void calculateIncrement() {
        //return approxLength;
    }
    public Bezier(double heading, Point... waypoints) {
        this.waypoints = waypoints;
        this.heading = normalizeDegrees(heading);
        generateCurve();
        this.headingFollower = true;
    }

    public Bezier(double heading, boolean headingFollower, Point... waypoints) {
        this.headingFollower = headingFollower;
        this.waypoints = waypoints;
        this.heading = normalizeDegrees(heading);
        generateCurve();
    }


    public Bezier(){}


    public Bezier(Point... waypoints) {
        this(0, waypoints);
    }

    public Point[] getWaypoints() {
        return waypoints;
    }

    public void generateCurve(){

        int len = (int)(1.0/defaultIncrement) + 1;

        curvePoints = new Point[len];
        curveDerivatives = new Point[len];
        curveHeadings = new double[len];

        double currentT = 0;

        for(int i=0;i<curvePoints.length;i++){
            curvePoints[i] = getPoint(currentT);
            curveDerivatives[i] = getDerivative(currentT);
            curveHeadings[i] = getHeading(currentT);

            currentT += defaultIncrement;
        }
        pointCount = len;
        double length = approximateLength();
        estimatedStopping = (int)(((length - endTrajThreshold)/length)/defaultIncrement);
    }

    public Point[] getCurvePoints(){return curvePoints;}
    public Point[] getCurveDerivatives(){return curveDerivatives;}
    public double[] getCurveHeadings(){return curveHeadings;}



    public double getHeading(double t){
//        if (t>= 1)
//            return heading;
        if (headingFollower)
            return Math.toDegrees(Math.atan2(getDerivative(t).getY(), getDerivative(t).getX()));
        else
            return this.heading;
    }


    public Point getPoint(double t){

        double x = 0;

        double y = 0;

        int n = waypoints.length-1;

        for (int i = 0; i <= n; i++) {

            double b = choose(n, i) * Math.pow((1-t), n-i) * Math.pow(t, i);
            x +=  b * waypoints[i].getX();

            y += b * waypoints[i].getY();

        }

        return new Point(x, y);

    }

    public Point getDerivative(double t){

        double x = 0;

        double y = 0;

        int n = waypoints.length-1;

        for (int i = 0; i <= n-1; i++) {

            double b = choose(n-1, i) * Math.pow((1-t), n-i-1) * Math.pow(t, i);

            x += b * (waypoints[i+1].getX()-waypoints[i].getX());
            y += b * (waypoints[i+1].getY()-waypoints[i].getY());
        }

        x *= n;
        y *= n;

        return new Point(x, y);

    }

//    public Point getDerivative2(double t){
//
//        double x = 0;
//
//        double y = 0;
//
//        int n = waypoints.length-1;
//
//        for (int i = 0; i <= n; i++) {
//
//            double b = choose(n, i);
//
//            if(n-i==0){
//                pow = Math.pow(t, i - 1);
//                x += b * i * pow * waypoints[i].getX();
//                y += b * i * pow * waypoints[i].getY();
//            }else if(i==0){
//                pow = Math.pow(1 - t, n - i - 1);
//                x += -1 * b * (n-i) * pow * waypoints[i].getX();
//                y += -1 * b * (n-i) * pow * waypoints[i].getY();
//            }else{
//                pow = Math.pow(1-t, n-i-1);
//                pow2 = Math.pow(t, i);
//                pow3 = Math.pow(1-t, n-i);
//                pow4 = Math.pow(t, i-1);
//
//                x += -1 * b * (n-i) * pow * pow2 * waypoints[i].getX() + b * pow3 * i * pow4 * waypoints[i].getX();
//                y += -1 * b * (n-i) * pow * pow2 * waypoints[i].getY() + b * pow3 * i * pow4 * waypoints[i].getY();
//            }
//
//
//        }
//
//        return new Point(x, y);
//
//    }


    public int choose(int n, int k) {
        return (factorial(n))/((factorial(k))*factorial(n-k));
    }

    public int factorial(int n) {
        if (n == 0)
            return 1;
        else
            return(n * factorial(n-1));
    }


    public double approximateLength() {
        double distance = 0;
        for(int i=5; i<=100; i+= 5) {
            double z = ((double) i)/100;
            Point pt = getPoint(z);
            Point pt2 = getPoint(z-0.05);
            distance += Math.hypot(pt.getX()-pt2.getX(), pt.getY()-pt2.getY());
        }

        return distance;
    }

    public double segmentLength(int index) {
        if (index > pointCount)
            return 0;
        double t = ((double)index)/((double)pointCount);
        Point p1 = getPoint(t);
        Point p2 = getPoint(t+defaultIncrement);
        return Math.hypot(p2.getX()-p1.getX(), p2.getY()-p1.getY());

    }

    public Point getEndPoint() {
        return waypoints[waypoints.length-1];
    }

    public Point getStartPoint(){return waypoints[0];}

}
