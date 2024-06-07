package org.firstinspires.ftc.teamcode.CommandBase;

import org.ejml.dense.fixed.MatrixFeatures_DDF2;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

public class MoveTillBackBoard extends Command {
    Localizer localizer;
    MotionPlanner mp;
    int mult;
    double currentDraw;
    double velocity;
    final double targetHeading;
    double addition = 10;
    boolean startedMoving;
    final double movementPower = 0.35;
    final double velocityThreshold = 0.5;
//    final double currentThreshold = 7.75;

    public MoveTillBackBoard(MotionPlanner mp, Localizer localizer, boolean red) {
        this.mp = mp;
        this.localizer = localizer;
        startedMoving = false;
        if (red) {
            mult = -1;
        }
        else {
            mult = 1;
        }
        targetHeading = mult*90;
        addition = addition*mult;
        velocity = 0;
    }

    @Override
    public void init() {
        mp.startTrajectory(new Bezier(-90,
                new Point(localizer.getX(), localizer.getY()),
                new Point(localizer.getX(), localizer.getY()+addition)
        ));

    }

    @Override
    public void update() {
        velocity = localizer.getAvgVelocity();
        if (velocity>=3) {
            startedMoving = true;
        }
    }
    @Override
    public boolean isFinished() {
        return (startedMoving && Math.abs(velocity)<=velocityThreshold);
    }
}
