package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.MainSailMove;
import org.firstinspires.ftc.teamcode.CommandBase.PosOuttakePixel;
import org.firstinspires.ftc.teamcode.CommandBase.PixelHolderFunc;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.vision.SpikeDetect;

//@Autonomous
public class AutoParent extends LinearOpMode {

    public enum StartPos {
        BLUE_IN,
        BLUE_OUT,
        RED_IN,
        RED_OUT
    }

    boolean red = false;
    StartPos startPos = StartPos.BLUE_IN;

    // camera stuff
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera
    SpikeDetect.Location location;
    Webcam.AprilTagLocation aprilTagLocation;

    Bezier spike;
    Bezier toAprilTags;

    private int transX;
    private int transY;
    private int transZ;

    private int rotateYaw;
    private int rotatePitch;
    private int rotateRoll;

    // command stuff
    /*
    * 1. move to spike
    * 2. lower arm + pixel holder
    * 3. spit out
    * 4. retract
    * 5. move to backboard
    * 6. move arm + pixel holder
    * 7. spit out
    * 8. park
    * */


    @Override
    public void runOpMode() throws InterruptedException {

        Gnocchi.init(hardwareMap);

        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

        spike = new Bezier(180,
                new Point(0, 0),
                new Point(11.3, 30.7),
                new Point(46, 20.5),
                new Point(47, 0));;

        toAprilTags = new Bezier(-90,
                new Point(47, 0),
                new Point(49, -10),
                new Point(52, -40),
                new Point(48, -42));

        SequentialCommand scheduler = new SequentialCommand(
                // move to position
                new ParallelCommand(
                        new FollowTrajectory(motionPlanner, spike),
                        new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.SPIKE.getPosition())),
                        new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.SPIKE.getPosition()))
                ),
                new Wait(100),
                new PixelHolderFunc(false, false),
                new Wait(500),
                new MainSailMove(MainSail.ArmPos.RETRACT.getPosition(), MainSail.HolderPos.RETRACT.getPosition()),
                new FollowTrajectory(motionPlanner, toAprilTags),
                // move to position
                // use april tags
                new PosOuttakePixel(),
                new PixelHolderFunc(false, false)
                // park
        );

//        Gnocchi.webcam.init(hardwareMap);
//        waitForStart();

        while (!isStarted() && !isStopRequested()) {

            spike = new Bezier(180,
                    new Point(0, 0),
                    new Point(11.3, 30.7),
                    new Point(46, 20.5),
                    new Point(47, 0));

            toAprilTags = new Bezier(-90,
                    new Point(47, 0),
                    new Point(49, -10),
                    new Point(52, -40),
                    new Point(48, -42));

//            Gnocchi.webcam.scanForLocation();
//            location = Gnocchi.webcam.getLocation();
//
//            if (location != null) {
//                telemetry.addData("Location: ", location);
//                telemetry.update();
//            }
//
//            switch (location) {
//                case RIGHT:
//                    spike = new Bezier(-150,
//                    new Point(0, 0),
//                    new Point(11.3, 30.7),
//                    new Point(46, 20.5),
//                    new Point(47, 0));
//                    break;
//                case MID:
//                    spike = new Bezier(180,
//                    new Point(0, 0),
//                    new Point(11.3, 30.7),
//                    new Point(46, 20.5),
//                    new Point(47, 0));
//                    break;
//                case LEFT:
//                    spike = new Bezier(150,
//                    new Point(0, 0),
//                    new Point(11.3, 30.7),
//                    new Point(46, 20.5),
//                    new Point(47, 0));
//                    break;
//            }
//
//            switch (startPos) {
//                case BLUE_IN:
//                    board = new HeadingFollowerPath();
//                    break;
//                case BLUE_OUT:
//                    board = new HeadingFollowerPath();
//                    break;
//                case RED_IN:
//                    board = new HeadingFollowerPath();
//                    break;
//                case RED_OUT:
//                    board = new HeadingFollowerPath();
//                    break;
//            }
//
//            motionPlanner.startTrajectory(spike);

        }

        scheduler.init();

        motionPlanner.startTrajectory(spike);
        while(opModeIsActive() && !isStopRequested()) {
            motionPlanner.update();
            Gnocchi.slides.update();
//            scheduler.update();

            telemetry.addData("index", scheduler.getIndex());
            telemetry.addData("finished", scheduler.isFinished());
            telemetry.update();
        }

//        Gnocchi.webcam.stopStreaming();
    }
}