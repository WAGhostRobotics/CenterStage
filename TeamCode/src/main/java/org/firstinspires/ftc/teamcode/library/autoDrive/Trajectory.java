package org.firstinspires.ftc.teamcode.library.autoDrive;

import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.PathPoint;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Segment;

public class Trajectory {
    Bezier bezier;
    Localizer localizer;
    double endVelocity;



    public Segment[] segments;
    double initialHeading;
    public Trajectory(Bezier bezier, Localizer localizer, double endVelocity) {
        this.localizer = localizer;
        this.bezier = bezier;
        this.segments = new Segment[bezier.pointCount-1];
        this.endVelocity = endVelocity;
        generateTrajectory();
    }

    private void generateTrajectory() {
        double tIncrement = 1.0/(bezier.pointCount-1);
        double t = tIncrement;
        PathPoint p0 = new PathPoint(bezier.getPoint(0), 0, localizer.getHeading());
        PathPoint p1 = new PathPoint(bezier.getPoint(t), bezier.getHeading(t));
        int segLen = segments.length;
        for (int i =0;i<segLen;i++) {
            segments[i] = new Segment(p0, p1);
            p0 = segments[i].pathPoints[segments[i].pathPoints.length-1];
            t += tIncrement;
            p1 = new PathPoint(bezier.getPoint(t), bezier.getHeading(t));
        }
        segments[segLen-1].pathPoints[segments[segLen-1].pathPoints.length-1].setVelocity(endVelocity);
//        segments[segLen-1].pathPoints[segments[segLen-1].pathPoints.length-1].setHeading(10);

        for (int i =segLen-1; i>=0; i--) {
            segments[i].reverseFF();
//            if (i!=0) {
//                int pathPointsLast = segments[i-1].pathPoints.length-1;
//                segments[i-1].pathPoints[pathPointsLast].setVelocity(segments[i].getLastVelocity());
//            }
        }
    }

}
