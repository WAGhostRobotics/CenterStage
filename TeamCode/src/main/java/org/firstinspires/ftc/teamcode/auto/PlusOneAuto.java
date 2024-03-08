package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.MainSailMove;
import org.firstinspires.ftc.teamcode.CommandBase.PosOuttakePixel;
import org.firstinspires.ftc.teamcode.CommandBase.PixelHolderFunc;
import org.firstinspires.ftc.teamcode.CommandBase.SlidesMove;
import org.firstinspires.ftc.teamcode.CommandBase.Spintake;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
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

public class PlusOneAuto extends LinearOpMode {

    boolean red = false;
    boolean left = false;
    int mult;

    // vision stuff
    SpikeDetect.Location location;
    Webcam.AprilTagLocation aprilTagLocation = red ? Webcam.AprilTagLocation.SIX : Webcam.AprilTagLocation.THREE;

    // path stuff
    Bezier spike;
    Bezier whitePix;
    Bezier scooch;
    Bezier toAprilTags;
    Bezier park;


    @Override
    public void runOpMode() throws InterruptedException {

        Gnocchi.init(hardwareMap, red, left);

        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

        Gnocchi.webcam.init(hardwareMap);

        spike = new Bezier(
                new Point(0, 0)
        );

        if (red) {
            mult = -1;
        } else {
            mult = 1;
        }

        SequentialCommand getPix = new SequentialCommand();

        while (opModeInInit()) {
            // get ready for switch/if statement hell

            location = Gnocchi.webcam.getLocation();
            Gnocchi.launcher.close();

            if (location != null) {
                telemetry.addData("Location: ", location);
                telemetry.update();

                // spike trajectories
                switch (location) {
                    case RIGHT:
                        if (red) {
                            spike = new Bezier(-90,
                                    new Point(0, 0),
                                    new Point(23.5, 0));
                        } else {
                            spike = new Bezier(20,
                                    new Point(0, 0),
                                    new Point(45, -1));
                            whitePix = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(50, 0),
                                    new Point(50, -12));
                            scooch = new Bezier(90,
                                    whitePix.getEndPoint(),
                                    new Point(50, -5));
                            getPix = new SequentialCommand(
                                    new RunCommand(() -> Gnocchi.intake.setHeight(Intake.IntakeHeight.PIXEL1)),
                                    new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition())),
                                    new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition())),
                                    new ParallelCommand(
                                            new SlidesMove(Slides.TurnValue.INTAKE.getTicks()),
                                            new Spintake(true, false),
                                            new PixelHolderFunc(false, true)
                                    ),
                                    new ParallelCommand(
                                            new FollowTrajectory(motionPlanner, scooch),
                                            new Spintake(false, true)
                                    ),
                                    new RunCommand(() -> Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT))
                            );
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.SIX : Webcam.AprilTagLocation.THREE;
                        break;

                    case MID:
                        if (red) {
                            spike = new Bezier(
                                    new Point(0, 0),
                                    new Point(40, 3));
                        } else {
                            spike = new Bezier(0,
                                    new Point(0, 0),
                                    new Point(50, 5));
                            whitePix = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(40, 0),
                                    new Point(50, -12));
                            scooch = new Bezier(90,
                                    whitePix.getEndPoint(),
                                    new Point(50, -6));
                            getPix = new SequentialCommand(
                                    new RunCommand(() -> Gnocchi.intake.setHeight(Intake.IntakeHeight.PIXEL1)),
                                    new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition())),
                                    new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition())),
                                    new ParallelCommand(
                                            new SlidesMove(Slides.TurnValue.INTAKE.getTicks()),
                                            new Spintake(true, false),
                                            new PixelHolderFunc(false, true)
                                    ),
                                    new ParallelCommand(
                                            new FollowTrajectory(motionPlanner, scooch),
                                            new Spintake(false, true)
                                    ),
                                    new RunCommand(() -> Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT))
                            );
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.FIVE : Webcam.AprilTagLocation.TWO;
                        break;

                    case LEFT:
                        if (red) {
                            spike = new Bezier(-90,
                                    new Point(0, 0),
                                    new Point(27, 0)
                            );
                        } else {
                            spike = new Bezier(-90,
                                    new Point(0, 0),
                                    new Point(23, 4),
                                    new Point(25, 20),
                                    new Point(27, 4)
                            );
                            whitePix = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(50, 0),
                                    new Point(50, -10));
                            scooch = new Bezier(90,
                                    whitePix.getEndPoint(),
                                    new Point(50, -5));
                            getPix = new SequentialCommand(
                                    new RunCommand(() -> Gnocchi.intake.setHeight(Intake.IntakeHeight.PIXEL1)),
                                    new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition())),
                                    new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition())),
                                    new ParallelCommand(
                                            new SlidesMove(Slides.TurnValue.INTAKE.getTicks()),
                                            new Spintake(true, false),
                                            new PixelHolderFunc(false, true)
                                    ),
                                    new ParallelCommand(
                                            new FollowTrajectory(motionPlanner, scooch),
                                            new Spintake(false, true)
                                    ),
                                    new RunCommand(() -> Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT))
                            );
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.FOUR : Webcam.AprilTagLocation.ONE;
                        break;
                }

                // april tag trajectories
                switch (aprilTagLocation) {
                    case ONE:
                        toAprilTags = new Bezier(90,
                                spike.getEndPoint(),
                                new Point(25, -22),
                                new Point(53, 30),
                                new Point(15, 40),
                                new Point(10, 92)
                        );
                        break;
                    case TWO:
                        toAprilTags = new Bezier(90,
                                spike.getEndPoint(),
                                new Point(45, 30),
                                new Point(13, 54),
                                new Point(6, 60),
                                new Point(22, 91.2)
                        );
                        break;
                    case THREE:
                        toAprilTags = new Bezier(90,
                                spike.getEndPoint(),
                                new Point(40, 30),
                                new Point(31, 55),
                                new Point(30, 89.5)
                        );
                        break;
                    case FOUR:
                        toAprilTags = new Bezier(-90,
                                spike.getEndPoint(),
                                new Point(60, 12),
                                new Point(45, -50),
                                new Point(31.5, -87.5)
                        );
                        break;
                    case FIVE:
                        toAprilTags = new Bezier(-90,
                                spike.getEndPoint(),
                                new Point(75, -35),
                                new Point(20, -39),
                                new Point(33.5, -86)
                        );
                        break;
                    case SIX:
                        toAprilTags = new Bezier(-90,
                                spike.getEndPoint(),
                                new Point(20, 1),
                                new Point(50, 5),
                                new Point(70, -74),
                                new Point(25, -89)
                        );
                        break;
                }
                park = new Bezier(mult * 90, toAprilTags.getEndPoint());

            }
        }

        SequentialCommand scheduler = new SequentialCommand(
                // spike
                new FollowTrajectory(motionPlanner, spike),
                new Wait(100),
                new Spintake(false, false),
                new Wait(400),

                new FollowTrajectory(motionPlanner, whitePix),
                getPix,

                // april tag
                new ParallelCommand(
                        new SequentialCommand(
                                new Wait(700),
                                new ParallelCommand(
                                        new MainSailMove(MainSail.ArmPos.RETRACT.getPosition(), MainSail.HolderPos.RETRACT.getPosition())
                                )
                        ),
                        new SequentialCommand(
                                new Wait(800),
                                new FollowTrajectory(motionPlanner, toAprilTags)
                        )
                ),
                new Wait(300),
                new PosOuttakePixel(),
                new Wait(300),
                new PixelHolderFunc(false, false),
                new MainSailMove(MainSail.ArmPos.RETRACT.getPosition(), MainSail.HolderPos.RETRACT.getPosition()),
                new SlidesMove(Slides.TurnValue.INTAKE.getTicks()),
                new FollowTrajectory(motionPlanner, park)
        );

        scheduler.init();
        motionPlanner.startTrajectory(spike);

        while (opModeIsActive() && !isStopRequested()) {
            motionPlanner.update();
            Gnocchi.slides.update();
            scheduler.update();

            telemetry.addData("index", scheduler.getIndex());
            telemetry.addData("finished", scheduler.isFinished());
            telemetry.addData("", motionPlanner.getTelemetry());
            telemetry.update();
        }

        Gnocchi.webcam.stopStreaming();
    }
}