package org.firstinspires.ftc.teamcode.library.autoDrive.math;

public class Segment {
    final double maxVel = 60; // inches per second
    final double maxAcc = 120; // inches per second

    final double maxDecc = -250; // inches per second
    // Might increase maxDecc
    // Set motor zero power behavior to brake

    public PathPoint[] pathPoints;
    double initialHeading;
    double targetHeading;
    double targetX;
    double targetY;
    double headingChange;
    public Segment(PathPoint p1, PathPoint p2) {
        initialHeading = p1.getHeading();
        targetHeading = p2.getHeading();
        targetX = p2.point.x;
        targetY = p2.point.y;
        headingChange = targetHeading-initialHeading;
        int len = (int) Math.ceil(Math.hypot(p2.point.getX()-p1.point.getX(), p2.point.getY()-p1.point.getY()));
        pathPoints = new PathPoint[len+1];
        pathPoints[0] = p1;
        generateFF();
    }

    public void generateFF() {
        for (int i = 1; i<pathPoints.length; i++) {
            double x = pathPoints[i-1].point.getX() + Math.cos(Math.toRadians(initialHeading)) * 1;
            double y = pathPoints[i-1].point.getY() + Math.sin(Math.toRadians(initialHeading)) * 1;
            double initialV = pathPoints[i-1].getVelocity();
            double finalV = getMaxReachableVelocity(initialV);
            double dt = ((-1*initialV) + Math.sqrt(Math.pow(initialV, 2) - (4*maxAcc*1)))
                    /(2*maxAcc);

            double acc = (finalV != initialV) ? maxAcc : 0;
            pathPoints[i] = new PathPoint(x, y, finalV, acc, initialHeading, 0, dt);
        }
        pathPoints[pathPoints.length-1].setVelocity(0);
//        pathPoints[pathPoints.length-1].setHeading(targetHeading);
        reverseFF();
    }
    public void reverseFF() {

        for (int i = pathPoints.length-1; i>0; i--) {
            double finalV = pathPoints[i].getVelocity();
            double initialV = pathPoints[i-1].getVelocity();
            double brakeDistance = -1*Math.pow(initialV, 2)/(2*maxDecc);
            if (Math.hypot(targetX-pathPoints[i].point.x, targetY-pathPoints[i].point.y) > brakeDistance) {
                pathPoints[i].setVelocity(0);
                pathPoints[i].setBrake(true);
                break;
            }
            pathPoints[i].setBrake(true);
            pathPoints[i].setVelocity(0);
        }
//        pathPoints[pathPoints.length-1].setBrake(false);
        // Setting last pathPoint brake to allow end tuner to function
        // Might be sketchy?
    }

    private double getMaxReachableVelocity(double currentV) {
        double reachableV = Math.sqrt(
                Math.pow(currentV, 2) + 2*maxAcc
        );
        return Math.min(maxVel, reachableV);
    }

    public double getLastVelocity() {
        return pathPoints[pathPoints.length-1].velocity;
    }

}
