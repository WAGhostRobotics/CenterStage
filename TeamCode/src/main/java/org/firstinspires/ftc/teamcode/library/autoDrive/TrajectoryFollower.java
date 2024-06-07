package org.firstinspires.ftc.teamcode.library.autoDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Segment;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;

public class TrajectoryFollower {
    public static double Kv = 0.0142; //0.0142
    public static double Ka = 0.0034; //0.0034
    public static double KStaticX = 0.2;
    public static double KStaticY = 0.5;
    public static double KStaticTurn = 0.08;
    public static double KStaticTurnEnd = 0.285;
    public static PIDController translationalControlX = new PIDController(0.02, 0.00001, 0);
    public static PIDController translationalControlY = new PIDController(0.01, 0, 0);
    public static PIDController headingControl = new PIDController(0.03, 0.00001, 0);
    public static PIDController headingControlEnd = new PIDController(0.0073, 0.00, 0);

    Drivetrain drive;
    Localizer localizer;
    Trajectory trajectory;
    double x;
    double y;
    double targetX;
    double targetY;
    double xError;
    double yError;
    double translationalError;
    double xPower;
    double yPower;
    double PIDMagnitude;
    double theta;
    double translationalPIDPower;
    double headingError;
    final double headingErrorThreshold = 1.5;
    final double translationalErrorThreshold= 1;
    double finalMagnitude;
    double targetHeading;
    double currentHeading;
    double brakeHeading;
    double acc;
    double driveTurn;
    int index;
    int brakeIndex;
    int segment;
    int totalSegments;
    int segmentLength;
    boolean end;
    boolean braked;

    Segment currentSeg;
    Segment nextSeg;
    HardwareMap hwMap;
    double voltage;
    double velocityThreshold = 1;
    double angularVThreshold = 1;


    public TrajectoryFollower(Drivetrain drive, Localizer localizer, Trajectory trajectory, HardwareMap hwMap) {
        this.drive = drive;
        this.localizer = localizer;
        this.trajectory = trajectory;
        this.end = false;
        headingControl.setIntegrationBounds(-10000000, 10000000);
        headingControlEnd.setIntegrationBounds(-10000000, 10000000);
        translationalControlX.setIntegrationBounds(-10000000, 10000000);
        translationalControlY.setIntegrationBounds(-10000000, 10000000);
        index = 1;
        segment = 0;
        currentSeg = trajectory.segments[segment];
        totalSegments = trajectory.segments.length;
        this.hwMap = hwMap;
        this.voltage = hwMap.voltageSensor.iterator().next().getVoltage();


    }

    public void followUpdate() {
        localizer.update();
        x = localizer.getX();
        y = localizer.getY();
        currentHeading = Math.toDegrees(localizer.getHeading());
        currentSeg = trajectory.segments[segment];

        // Get the right index in the segment
        while (index<currentSeg.pathPoints.length-1) {
            if (distance(currentSeg.pathPoints[index+1].point,  new Point(x, y)) <
                distance(currentSeg.pathPoints[index].point, new Point(x, y))) {
                index++;
            }
            else
                break;
        }
        targetX = currentSeg.pathPoints[index].point.getX();
        targetY = currentSeg.pathPoints[index].point.getY();
        targetHeading = currentSeg.pathPoints[index].getHeading();

        // Get translational error
        xError = targetX - x;
        yError = targetY - y;
        translationalError = Math.hypot(xError, yError);
        headingError = getHeadingError();

        // Get Power using Just PID
        theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);

        xError = Math.cos(Math.toRadians(theta))*translationalError;
        yError = Math.sin(Math.toRadians(theta))*translationalError;
        xPower = translationalControlX.calculate(0, xError);
        yPower = translationalControlY.calculate(0, yError);

        xPower = xPower + Math.signum(xPower)* KStaticX;
        yPower = yPower + Math.signum(yPower)* KStaticY;
        PIDMagnitude = Math.hypot(xPower, yPower);

        if (currentSeg.pathPoints[index].getBrake() && Math.abs(localizer.getAvgVelocity())>velocityThreshold && !braked) {
            index = currentSeg.pathPoints.length-1;
            if (!end) {
                brakeHeading = currentHeading;
                brakeIndex = index;
            }
            driveTurn = headingControl.calculate(0, headingError);
            driveTurn = (!reachedHeadingTarget()) ? driveTurn + Math.signum(driveTurn)*KStaticTurn : 0;
            drive.brake(localizer, driveTurn, 0.85);
//            drive.brake(localizer, 0.85);
            end = true;
        }
        else {
            if (end) {
                driveTurn = headingControlEnd.calculate(0, headingError);
                braked = true;
                driveTurn = driveTurn + Math.signum(driveTurn)* KStaticTurnEnd;
            }
            else {
                driveTurn = headingControl.calculate(0, headingError);
                driveTurn = driveTurn + Math.signum(driveTurn)* KStaticTurn;
            }
            if (reachedTranslationalTarget()) {
                PIDMagnitude = 0;
                theta = 0;
                translationalControlX.reset();
                translationalControlY.reset();
            }
            if (reachedHeadingTarget()) {
                driveTurn = 0;
                headingControl.reset();
                headingControlEnd.reset();
            }

            finalMagnitude = currentSeg.pathPoints[index].getVelocity() * Kv +
                    currentSeg.pathPoints[index].getAcc() * Ka + PIDMagnitude;

            drive.drive(finalMagnitude, theta, driveTurn, 0.85, voltage);

            if (isFinished()) {
                if (segment < totalSegments-1) {
                    segment++;
                    index = 1;
                    end = false;
                    braked = false;
                    headingControl.reset();
                    headingControlEnd.reset();
                    translationalControlX.reset();
                    translationalControlY.reset();
                }
            }
        }
    } // Add movement Kstatic for End as 0.22

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
    private boolean reachedHeadingTarget() {
        return (Math.abs(headingError) <= headingErrorThreshold);
    }
    private boolean isFinished() {
        return (reachedHeadingTarget() && reachedTranslationalTarget() &&
                end && localizer.getAvgVelocity()<velocityThreshold
                && localizer.getAngularVelocityImu()< angularVThreshold);
    }
    private boolean reachedTranslationalTarget() {
        return (Math.abs(Math.hypot(xError, yError))<=translationalErrorThreshold);
    }
    public String getTelemetry() {
        return "Index: " + index +
                "\nSegment: " + segment +
                "\nTotal Segments: " + trajectory.segments.length +
                "\nTotal Indices: " + currentSeg.pathPoints.length +
                "\n\nActual values:" +
                "\nVelocity: " + localizer.getAvgVelocity() +
                "\nHeading: " + currentHeading +
                "\nX: " + x +
                "\nY: " + y +
                "\n\nSet values: " +
                "\nMagnitude: " + finalMagnitude +
                "\nDriveTurn: " + driveTurn +
                "\nTheta: " + theta +
                "\nTarget X: " + currentSeg.pathPoints[index].point.getX() +
                "\nTarget Y: " + currentSeg.pathPoints[index].point.getY() +
                "\nTarget Velocity: " + currentSeg.pathPoints[index].getVelocity() +
                "\nTarget Acc: " + currentSeg.pathPoints[index].getAcc() +
                "\nTarget Heading: " + targetHeading +
                "\nTranslational PID Power: " + PIDMagnitude +
                "\n\nErrors:" +
                "\nx Error: " + xError +
                "\ny Error: " + yError +
                "\nDistance next/Total Error: " + distance(currentSeg.pathPoints[index].point,  new Point(x, y)) +
                "\nHeading error: " + headingError +
                "\nBrake: " + currentSeg.pathPoints[index].getBrake() +
                "\nReached Heading Target: " + reachedHeadingTarget() +
                "\nReached Translational Target: " + reachedTranslationalTarget() +
                "\nEnd: " + end +
                "\nBrake Index: " + brakeIndex +
                "\nIsFinished: " + isFinished();

    }

}
