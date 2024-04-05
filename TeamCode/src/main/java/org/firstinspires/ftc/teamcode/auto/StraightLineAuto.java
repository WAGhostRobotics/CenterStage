package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.Trajectory;
import org.firstinspires.ftc.teamcode.library.autoDrive.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;

@Autonomous
public class StraightLineAuto extends LinearOpMode {
    Bezier curvyTest;
    Bezier straightTest;
    Bezier strafeTest;
    Bezier notSoStraightTest;
    Trajectory traj;
    TrajectoryFollower trajectoryFollower;
    @Override
    public void runOpMode() throws InterruptedException {

        Gnocchi.init(hardwareMap);
        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

//        SequentialCommand scheduler = new SequentialCommand(
//                new FollowTrajectory(motionPlanner, curvyTest)
//                //new IntakePixel()
//        );
//        scheduler.init();
        //motionPlanner.startTrajectory(straightTest);
        while (opModeInInit()) {
            curvyTest = new Bezier(
                    -90,
                    new Point(0, 0),
                    new Point(4.6, 50.3),
                    new Point(138, 56.5),
                    new Point(-54, -61.4),
                    new Point(5, 33.6),
                    new Point(27, 19)
            );
            straightTest = new Bezier(
                    0,
                    new Point(0, 0),
                    new Point(80, 0)
            );
            notSoStraightTest = new Bezier(
                    0,
                    new Point(0, 0),
                    new Point(20, 20),
                    new Point(40, 10),
                    new Point(60, -10),
                    new Point(80, 0)
            );
            strafeTest = new Bezier(
                    0,
                    new Point(0, 0),
                    new Point(0, 80)
            );
            traj = new Trajectory(notSoStraightTest, localizer, 0);
            trajectoryFollower = new TrajectoryFollower(drive, localizer, traj);
            telemetry.addData("", trajectoryFollower.getTelemetry());
            telemetry.update();
        }
        while(opModeIsActive() && !isStopRequested()) {
            //motionPlanner.update();
//            scheduler.update();
//            motionPlanner.update();
            trajectoryFollower.followUpdate();
            telemetry.addData("", trajectoryFollower.getTelemetry());
            telemetry.addData("Heading: ", normalizeDegrees(localizer.getHeading(Localizer.Angle.DEGREES)));
            telemetry.update();
        }
    }

}
