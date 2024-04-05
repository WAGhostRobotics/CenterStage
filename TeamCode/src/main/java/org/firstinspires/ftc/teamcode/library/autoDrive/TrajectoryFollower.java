package org.firstinspires.ftc.teamcode.library.autoDrive;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Segment;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;

public class TrajectoryFollower {
    public static double Kv = 0.0142;
    public static double Kstatic = 0.01;
    public static double Ka = 0.0034;

    public static double Kang = 0.00;
    public static PIDController translationalControl = new PIDController(0.01, 0.0001, 0);
    public static PIDController headingControl = new PIDController(0.0073, 0.000001, 0);

    Drivetrain drive;
    Localizer localizer;
    Trajectory trajectory;
    double x;
    double y;
    double targetVelocity;
    double targetHeading;
    double currentHeading;
    double acc;
    double driveTurn;
    int index;
    int segment;
    int totalSegments;
    int segmentLength;

    Segment currentSeg;
    Segment nextSeg;


    public TrajectoryFollower(Drivetrain drive, Localizer localizer, Trajectory trajectory) {
        this.drive = drive;
        this.localizer = localizer;
        this.trajectory = trajectory;
        headingControl.setIntegrationBounds(-10000000, 10000000);
        translationalControl.setIntegrationBounds(-10000000, 10000000);
        index = 1;
        segment = 0;
        currentSeg = trajectory.segments[segment];
        totalSegments = trajectory.segments.length;

    }

    public void followUpdate() {
        localizer.update();
        x = localizer.getX();
        y = localizer.getY();
        currentHeading = localizer.getHeading();

        while (segment < totalSegments-1) {
            currentSeg = trajectory.segments[segment];
            nextSeg = trajectory.segments[segment+1];
            int pathPointsCountCur = currentSeg.pathPoints.length;

            if (distance(new Point(x, y), currentSeg.pathPoints[pathPointsCountCur-1].point) >
                    distance(new Point(x, y), nextSeg.pathPoints[1].point)) {
                segment++;
                currentSeg = trajectory.segments[segment];
                index = 1;
            }
            else
                break;
        }
        while (index<currentSeg.pathPoints.length-1) {
            if (distance(currentSeg.pathPoints[index+1].point,  new Point(x, y)) <
                distance(currentSeg.pathPoints[index].point, new Point(x, y))) {
                index++;
            }
            else
                break;
        }

        targetVelocity = currentSeg.pathPoints[index].getVelocity() * Kv +
                currentSeg.pathPoints[index].getAcc() * Ka;
        targetVelocity = targetVelocity + Math.signum(targetVelocity)*Kstatic;
        targetHeading = currentSeg.pathPoints[index].getHeading();
        driveTurn = headingControl.calculate(0, getHeadingError());
        drive.drive(targetVelocity, 0, driveTurn, 0.85);
    }

    private double distance(Point p1, Point p2){
        return Math.hypot(p1.getX()-p2.getX(), p1.getY()-p2.getY());
    }
    public double getHeadingError(){
        double headingError = targetHeading - currentHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }

        return headingError;
    }
    public String getTelemetry() {
        return "Index: " + index +
                "\nSegment: " + segment +
                "\nTotoal Segments: " + trajectory.segments.length +
                "\nTotoal Indices: " + currentSeg.pathPoints.length +
                "\nVelocity: " + localizer.getAvgVelocity() +
                "\nHeading: " + localizer.getHeading() +
                "\nDriveturn: " + driveTurn +
                "\nTarget Velocity: " + targetVelocity +
                "\nTarget Heading: " + targetHeading +
                "\nAcc Index: " + currentSeg.pathPoints[index].getAcc() +
                "\nTarget X: " + currentSeg.pathPoints[index].point.getX() +
                "\nTarget Y: " + currentSeg.pathPoints[index].point.getY() +
                "\nX: " + x +
                "\nY: " + y +
                "\nDistance next: " + distance(currentSeg.pathPoints[index].point,  new Point(x, y)) +
                "\nDt: " + currentSeg.pathPoints[index].getDt();

    }

}
