package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

public class MoveTillPixel extends Command {
    Localizer localizer;
    MotionPlanner mp;
    int mult;
    double currentDraw;
    double velocity;
    final double targetHeading;
    double addition = 10.75;
    boolean pixelAcquired;
    double currentThreshold;
    ElapsedTime timer;

    private final double waitBeforeReading = 500;
    final double timeAfterAcquisition = 2000; //ms

    public MoveTillPixel(MotionPlanner mp, Localizer localizer, boolean red) {
        this.mp = mp;
        this.localizer = localizer;
        pixelAcquired = false;
        if (red) {
            mult = -1;
        }
        else {
            mult = 1;
        }
        targetHeading = mult*90;
        addition = addition*mult;
    }

    @Override
    public void init() {
        Gnocchi.intake.setHeight(Intake.IntakeHeight.PIXEL1);
        Gnocchi.intake.in();
        Gnocchi.mainSail.pixelDrop.setPower(1);
        timer = new ElapsedTime();
        double time = 0;
        while (time<=waitBeforeReading) {
            time = timer.milliseconds();
        }
        timer.reset();
        currentThreshold = Gnocchi.intake.getMotorCurrentDraw() + 0.8;
        mp.startTrajectory(new Bezier(targetHeading,
                new Point(localizer.getX(), localizer.getY()),
                new Point(localizer.getX(), localizer.getY()-addition)
        ));
    }

    @Override
    public void update() {
        currentDraw = Gnocchi.intake.getMotorCurrentDraw();
        if (!pixelAcquired && currentDraw>=currentThreshold) {
            timer = new ElapsedTime();
            timer.reset();
            pixelAcquired = true;
        }
    }
    @Override
    public boolean isFinished() {
        if (pixelAcquired && timer.milliseconds() >= timeAfterAcquisition) {
            Gnocchi.intake.stop();
            Gnocchi.mainSail.pixelDrop.setPower(0);
            return true;
        }
        return false;
    }
}
