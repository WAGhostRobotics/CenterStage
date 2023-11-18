package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;

@Autonomous(name = "ParkAuto")
public class ParkAuto extends LinearOpMode {

    // Sadge auto

    public enum StartPos {
        BLUE_IN,
        BLUE_OUT,
        RED_IN,
        RED_OUT
    }

    boolean red = false;
    StartPos startPos = StartPos.BLUE_IN;

    Localizer localizer = new Localizer(hardwareMap);
    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap);

    MotionPlanner motionPlanner = new MotionPlanner(mecanumDrive, localizer, hardwareMap);
    Bezier park;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (!isStarted() && !isStopRequested()) {

            switch (startPos) {
                case BLUE_IN:
                    park = new Bezier(new Point(0, 0), new Point(0, 50));
                case BLUE_OUT:
                    park = new Bezier(new Point(0, 0), new Point(0, 95));
                case RED_IN:
                    park = new Bezier(new Point(0, 0), new Point(0, 50));
                case RED_OUT:
                    park = new Bezier(new Point(0, 0), new Point(0, 95));
            }

        }

        while(opModeIsActive()) {
            // PARK OH YEAH
            motionPlanner.startTrajectory(park);
        }

    }
}