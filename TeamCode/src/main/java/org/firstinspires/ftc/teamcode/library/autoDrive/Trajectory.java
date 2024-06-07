package org.firstinspires.ftc.teamcode.library.autoDrive;

import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.PathPoint;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Segment;

public class Trajectory {
    PathPoint[] pathPoints;
    double endVelocity;
    double targetHeading;



    public Segment[] segments;
    double initialHeading;
//    public Trajectory(double endVelocity, PathPoint... pathPoints) {
//        this.pathPoints = pathPoints;
//        this.segments = new Segment[pathPoints.length-1];
//        this.endVelocity = endVelocity;
//        generateTrajectory();
//    }
    public Trajectory(double heading, PathPoint... pathPoints) {
        this.pathPoints = pathPoints;
        this.segments = new Segment[pathPoints.length-1];
        this.endVelocity = 0;
        this.targetHeading = heading;
        generateTrajectory();
    }

    private void generateTrajectory() {
        int segLen = segments.length;
        for (int i =0;i<segLen;i++) {
            PathPoint p0 = pathPoints[i];
            PathPoint p1 = pathPoints[i+1];
            p0.setHeading(Math.toDegrees(
                    Math.atan2(p1.point.getY()-p0.point.getY(), p1.point.getX()-p0.point.getX())));
            segments[i] = new Segment(p0, p1);
        }
        int i=0;
        for (; i<segLen-1; i++) {
            int lastPathPoint = segments[i].pathPoints.length-1;
            segments[i].pathPoints[lastPathPoint].setHeading(
                    segments[i+1].pathPoints[0].getHeading()
            );
        }
        segments[i].pathPoints[segments[i].pathPoints.length-1].setHeading(targetHeading);
    }

}
