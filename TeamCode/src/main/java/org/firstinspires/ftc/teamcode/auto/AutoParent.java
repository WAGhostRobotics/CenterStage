package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.MainSailMove;
import org.firstinspires.ftc.teamcode.CommandBase.PosOuttakePixel;
import org.firstinspires.ftc.teamcode.CommandBase.PixelHolderFunc;
import org.firstinspires.ftc.teamcode.CommandBase.SlidesMove;
import org.firstinspires.ftc.teamcode.CommandBase.Spintake;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.MergedBezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.vision.SpikeDetect;

public class AutoParent extends LinearOpMode {

    public enum StartPos {
        IN,
        OUT
    }

    boolean red = false;
    boolean left = false;
    int mult;
    StartPos startPos = StartPos.IN;

    // vision stuff
    SpikeDetect.Location location;
    Webcam.AprilTagLocation aprilTagLocation = red ? Webcam.AprilTagLocation.SIX : Webcam.AprilTagLocation.THREE;

    // path stuff
    Bezier spike;
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
                            if (startPos == StartPos.IN) {
                                spike = new MergedBezier(
                                        new Bezier(-90,
                                                new Point(0, 0),
                                                new Point(27, 0)
                                        ),
                                        new Bezier(-90,
                                                new Point(27, 0),
                                                new Point(27, -17.2)
                                        )
                                );
                            } else {
                                spike = new Bezier(-90,
                                        new Point(0, 0),
                                        new Point(23.5, 0));
                            }
                        } else {
                            if (startPos == StartPos.IN) {
                                spike = new Bezier(
                                        90,
                                        new Point(0, 0),
                                        new Point(25, 0)
                                );
                            } else {
                                spike = new Bezier(-90,
                                        new Point(0, 0),
                                        new Point(33, -29));
                            }
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.SIX : Webcam.AprilTagLocation.THREE;
                        break;

                    case MID:
                        if (red) {
                            if (startPos == StartPos.IN) {
//                                spike = new Bezier(170,
//                                        new Point(0, 0),
//                                        new Point(21, 5),
//                                        new Point(40, 5),
//                                        new Point(21, 7));
                                spike = new Bezier(0,
                                        new Point(0, 0),
                                        new Point(48.35, 6)
                                ); //TODO:stab clara stab sanjay
                            } else {
                                spike = new Bezier(
                                        new Point(0, 0),
                                        new Point(40, 3));
                            }
                        } else {
                            if (startPos == StartPos.IN) {
                                spike = new Bezier(0,
                                        new Point(0, 0),
                                        new Point(48.35, -5));
                            } else {
                                spike = new Bezier(0,
                                        new Point(0, 0),
                                        new Point(40, 3));
                            }
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.FIVE : Webcam.AprilTagLocation.TWO;
                        break;

                    case LEFT:
                        if (red) {
                            if (startPos == StartPos.IN) {
                                spike = new Bezier(
                                        -90,
                                        new Point(0, 0),
                                        new Point(25, 0)
                                );
                            } else {
                                spike = new Bezier(
                                        -90,
                                        new Point(0, 0),
                                        new Point(27, 0)
                                );
                            }
                        } else {
                            if (startPos == StartPos.IN) {
                                spike = new MergedBezier(
                                        new Bezier(90,
                                                new Point(0, 0),
                                                new Point(27, 0)
                                        ),
                                        new Bezier(90,
                                                new Point(27, 0),
                                                new Point(30, 25)
                                        )
                                );
                            } else {
                                spike = new Bezier(
                                        -90,
                                        new Point(0, 0),
                                        new Point(27, -5)
                                );
                            }
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.FOUR : Webcam.AprilTagLocation.ONE;
                        break;
                }

                // april tag trajectories
                switch (aprilTagLocation) {
                    case ONE:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(25, -20),
                                    new Point(58, 30),
                                    new Point(50, 58),
                                    new Point(17.5, 80)
                            );
                        } else {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(25, 17.2),
                                    new Point(16, 36)
                            );
                        }
                        break;
                    case TWO:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(45, 30),
                                    new Point(30, 48),
                                    new Point(16.7, 79)
                            );
                        } else {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(42, 24),
                                    new Point(22, 26),
                                    new Point(19.5, 35.5)
                            );
                        }
                        break;
                    case THREE:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(40, 30),
                                    new Point(30, 48),
                                    new Point(23, 79)
                            );
                        } else {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(23, 2),
                                    new Point(42, 22),
                                    new Point(29, 38)
                            );
                        }
                        break;
                    case FOUR:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(60, 12),
                                    new Point(45, -50),
                                    new Point(31.5, -87.5)
                            );
                        } else {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(23, 6),
                                    new Point(42, -22),
                                    new Point(32, -34)
                            );
                        }
                        break;
                    case FIVE:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(75, -35),
                                    new Point(20, -39),
                                    new Point(33.5, -86)
                            );
                        } else {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
//                                    new Point(20, -22),
//                                    new Point(60, -25),
//                                    new Point(28.5, -31)
                                    new Point(60, -20),
                                    new Point(29.4, -37) //TODO: toast cclara
                            );
                        }
                        break;
                    case SIX:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(20, 1),
                                    new Point(50, 5),
                                    new Point(70, -74),
                                    new Point(25, -89)
                            );
                        } else {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(50, -25),
                                    new Point(20.5, -32)
                            );
                        }
                        break;
                }
                if (startPos == StartPos.IN && !red) {
                    park = new Bezier(0, toAprilTags.getEndPoint(), new Point(1, 35.5));
                } else if (startPos == StartPos.IN) {
                    park = new Bezier(0, toAprilTags.getEndPoint(), new Point(0, 0));
                } else {
                    park = new Bezier(mult * 90, toAprilTags.getEndPoint());

                }
            }
        }


            SequentialCommand scheduler = new SequentialCommand(
                    // spike
                    new FollowTrajectory(motionPlanner, spike),
                    new Wait(100),
                    new Spintake(false),
                    new Wait(400),

                    // april tag
                    new ParallelCommand(
                            new SequentialCommand(
                                    new Wait(700),
                                    new ParallelCommand(
                                            new PixelHolderFunc(false, true),
                                            new MainSailMove(MainSail.ArmPos.RETRACT.getPosition(), MainSail.HolderPos.RETRACT.getPosition())
                                    )
                            ),
                            new SequentialCommand(
                                    new Wait(800),
                                    new FollowTrajectory(motionPlanner, toAprilTags)
                            )
                    ),
                    new Wait(700),
                    new PosOuttakePixel(),
                    new Wait(500),
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