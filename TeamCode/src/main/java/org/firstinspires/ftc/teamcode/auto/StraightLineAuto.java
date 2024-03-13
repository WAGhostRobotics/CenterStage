package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.IntakePixel;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;

@Autonomous
public class StraightLineAuto extends LinearOpMode {
    Bezier straightTest;

    @Override
    public void runOpMode() throws InterruptedException {

        Gnocchi.init(hardwareMap);
        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);
        straightTest = new Bezier(
                0,
                new Point(0, 0),
                new Point(0, 50),
                new Point(30, 30)
        );

        SequentialCommand scheduler = new SequentialCommand(
                new FollowTrajectory(motionPlanner, straightTest)
                //new IntakePixel()
        );
        scheduler.init();
        //motionPlanner.startTrajectory(straightTest);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            //motionPlanner.update();
            scheduler.update();
            telemetry.addData("", motionPlanner.getTelemetry());
            telemetry.update();
        }
    }

}
