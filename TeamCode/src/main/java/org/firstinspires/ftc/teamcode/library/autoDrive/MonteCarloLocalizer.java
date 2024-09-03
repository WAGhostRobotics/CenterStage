package org.firstinspires.ftc.teamcode.library.autoDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.autoDrive.math.Particle;
import org.firstinspires.ftc.teamcode.util.Encoder;
import java.util.Random;

public class MonteCarloLocalizer extends Localizer {
    private Encoder parallelEncoder;
    private Encoder perpendicularEncoder;
    private static final int  PARTICLES = 1000; // Number of particles
    private static final double initialVariance = 0.5; // Initial variance for particle distribution
    private static final double motionNoise = 0.1; // Noise in the motion model
    private static final double sensorNoise = 0.2; // Noise in the sensor model
    private static Random rand = new Random();



    Particle[] particles;

    public static double PERPENDICULAR_X = 3;
    public static double PARALLEL_Y = 0;
    public MonteCarloLocalizer(LinearOpMode linearOpMode, HardwareMap hwMap) {
        super(linearOpMode, hwMap, true);
        particles = new Particle[PARTICLES];
        double startingX = 0.0;
        double startingY = 0.0;
        double startingTheta = 0.0;
        for (int i = 0; i < PARTICLES; i++) {
            double x = startingX + rand.nextGaussian() * initialVariance;
            double y = startingY + rand.nextGaussian() * initialVariance;
            double theta = startingTheta + rand.nextGaussian() * initialVariance;
            particles[i] = new Particle(x, y, theta, 1.0 / PARTICLES);
        }
    }

    void predict(double deltaX, double deltaY, double deltaTheta) {
        for (Particle p : particles) {
            p.x += deltaX + rand.nextGaussian() * motionNoise;
            p.y += deltaY + rand.nextGaussian() * motionNoise;
            p.theta += deltaTheta + rand.nextGaussian() * motionNoise;
        }
    }

    void updateWeights(double[] sensorData) {
        for (Particle p : particles) {
            double expectedSensorReading = getExpectedSensorReading(p);
            p.weight = calculateWeight(sensorData, expectedSensorReading);
        }
    }
    double calculateWeight(double[] sensorData, double expectedSensorReading) {
        // Implement the sensor model, e.g., Gaussian distribution
        double error = sensorData[0] - expectedSensorReading;
        return Math.exp(-(error * error) / (2 * sensorNoise * sensorNoise));
    }

    double getExpectedSensorReading(Particle p) {
        // Implement this function to calculate the expected sensor reading for particle p
        // based on the known map and the particle's position.
        return 0; // Placeholder
    }

    Particle estimatePosition() {
        double x = 0, y = 0, theta = 0;
        for (Particle p : particles) {
            x += p.x;
            y += p.y;
            theta += p.theta;
        }
        return new Particle(x / particles.length, y / particles.length, theta / particles.length, 1.0);
    }


}
