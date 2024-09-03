package org.firstinspires.ftc.teamcode.library.autoDrive.math;

public class Particle {
    public double x;
    public double y;
    public double theta; // orientation
    public double weight;

    public Particle(double x, double y, double theta, double weight) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.weight = weight;
    }
}

