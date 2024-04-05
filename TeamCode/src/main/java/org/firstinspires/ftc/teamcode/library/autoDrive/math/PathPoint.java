package org.firstinspires.ftc.teamcode.library.autoDrive.math;

public class PathPoint {
    double heading;
    double velocity;
    double angularV;
    double dt;
    double acc;

    public Point point;
    public PathPoint(double x, double y, double velocity, double acc, double heading, double angularV, double dt) {
        point = new Point(x, y);
        this.velocity = velocity;
        this.heading = heading;
        this.acc = acc;
        this.angularV = angularV;
        this.dt = dt;
    }

    public PathPoint(Point p, double heading) {
        point = new Point(p.getX(), p.getY());
        this.heading = heading;

    }
    public PathPoint (PathPoint p) {
        this.point = new Point(p.point.getX(), p.point.getY());
        this.velocity = p.getVelocity();
        this.acc = p.getAcc();
        this.heading = p.getHeading();
        this.angularV = p.getAngularV();
        this.dt = p.getDt();
    }

    public PathPoint(Point p, double velocity, double heading) {
        point = new Point(p.getX(), p.getY());
        this.heading = heading;
        this.velocity = velocity;

    }
    public void setX(double x) {
        this.point.x = x;
    }
    public void setY(double y) {
        this.point.y = y;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }
    public double getVelocity() {
        return velocity;
    }
    public void setAcc(double acc) {
        this.acc = acc;
    }

    public double getAcc() {
        return acc;
    }

    public double getHeading() {
        return heading;
    }
    public double getDt() {
        return dt;
    }

    public double getAngularV() {
        return angularV;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void setAngularV(double angularV) {
        this.angularV = angularV;
    }
}
