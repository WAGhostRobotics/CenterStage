package org.firstinspires.ftc.teamcode.library.autoDrive.math;

public class Segment {
    final double maxVel = 60; // inches per second
    final double maxAcc = 120; // inches per second

    final double maxDecc = 75; // inches per second
    // Might increase maxDecc
    // Set motor zero power behavior to brake

    public PathPoint[] pathPoints;
    double initialHeading;
    double targetHeading;
    double headingChange;
    public Segment(PathPoint p1, PathPoint p2) {
        initialHeading = p1.getHeading();
        targetHeading = p2.getHeading();
        headingChange = targetHeading-initialHeading;
        int len = (int) Math.hypot(p2.point.getX()-p1.point.getX(), p2.point.getY()-p1.point.getY());
        pathPoints = new PathPoint[len+1];
        pathPoints[0] = p1;
        generateFF();
    }

    public void generateFF() {
        for (int i = 1; i<pathPoints.length; i++) {
            double x = pathPoints[i-1].point.getX() + Math.cos(Math.toRadians(targetHeading)) * 1;
            double y = pathPoints[i-1].point.getY() + Math.sin(Math.toRadians(targetHeading)) * 1;
            double initialV = pathPoints[i-1].getVelocity();
            double finalV = getMaxReachableVelocity(initialV);
            double dt = ((-1*initialV) + Math.sqrt(Math.pow(initialV, 2) - (4*maxAcc*1)))
                    /(2*maxAcc);

            double acc = (finalV != initialV) ? maxAcc : 0;
            pathPoints[i] = new PathPoint(x, y, finalV, acc, targetHeading, 0, dt);
        }
//        reverseFF();
    }
    public void reverseFF() {

        for (int i = pathPoints.length-1; i>0; i--) {
            double finalV = pathPoints[i].getVelocity();
            double initialV = pathPoints[i-1].getVelocity();
            double dt = pathPoints[i].getDt();
            if (initialV > finalV- (maxDecc*dt)) {
                initialV = finalV - (maxDecc*dt);
            }
            dt = ((-1*initialV) + Math.sqrt(Math.pow(initialV, 2) - (4*maxAcc*1)))
                    /(2*maxAcc);
            pathPoints[i-1].setVelocity(initialV);
            pathPoints[i].setDt(dt);
        }
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

    private double headingChange() {
        return targetHeading-initialHeading;
    }

}
