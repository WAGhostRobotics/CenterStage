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
    Bezier bruh;


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
                                                new Point(28.5, -18)  //y was -17.2
                                        )
                                );
                            } else {
                                spike = new Bezier(90,
                                        new Point(0, 0),
                                        new Point(27, -12),
                                        new Point(27, -2));
                            }
                        } else {
                            if (startPos == StartPos.IN) {
                                spike = new Bezier(
                                        90,
                                        new Point(0, 0),
                                        new Point(25.5, -12),
                                        new Point(27.5, 0)
                                );
                            } else {
                                spike = new Bezier(15,
                                        new Point(0, 0),
                                        new Point(40, -10),
                                        new Point(44.5, -1.7));
                            }
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.SIX : Webcam.AprilTagLocation.THREE;
                        break;

                    case MID:
                        if (red) {
                            if (startPos == StartPos.IN) {
                                spike = new Bezier(0,
                                        new Point(0, 0),
                                        new Point(49, 6)
                                ); //TODO:stab clara stab sanjay
                            } else {
                                spike = new Bezier(
                                        new Point(0, 0),
                                        new Point(49.5, -0.8));
                            }
                        } else {
                            if (startPos == StartPos.IN) {
                                spike = new Bezier(0,
                                        new Point(0, 0),
                                        new Point(47.7, -5));
                            } else {
                                spike = new Bezier(0,
                                        new Point(0, 0),
                                        new Point(49, 6));
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
                                        new Point(24, 16),
                                        new Point(25, 2)    //y was 4 before i came along
                                );
                            } else {
                                spike = new Bezier(
                                        -90,
                                        new Point(0, 0),
                                        new Point(26, 15),
                                        new Point(27, -6)
                                );
                            }
                        } else {
                            if (startPos == StartPos.IN) {
                                spike = new MergedBezier(
                                        new Bezier(70,
                                                new Point(0, 0),
                                                new Point(27, 0)
                                        ),
                                        new Bezier(90,
                                                new Point(27, 0),
                                                new Point(28, 25)
                                        )
                                );
                            } else {
                                spike = new Bezier(
                                        -90,
                                        new Point(0, 0),
                                        new Point(20, 4),
                                        new Point(23, 20),
                                        new Point(25, 4)
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
                            toAprilTags = new MergedBezier(
                                    new Bezier(160,
                                            spike.getEndPoint(),
                                            new Point(30, -11),
                                            new Point(40, -5),
                                            new Point(52, 60),
                                            new Point(22, 70)
                                    ),
                                    new Bezier(120,
                                            new Point(22, 70),
                                            new Point(0.5, 88.8)
                                    )
                            );
                        } else {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(25, 19.7),
                                    new Point(15, 36.5)
                            );
                        }
                        break;
                    case TWO:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(100,
                                    spike.getEndPoint(),
                                    new Point(46.3, 4),
                                    new Point(65, 28),
                                    new Point(20, 50),
                                    new Point(17.5, 88.5)
                            );
                        } else {
                            toAprilTags = new Bezier(100,
                                    spike.getEndPoint(),
                                    new Point(42, 24),
                                    new Point(19, 27),
                                    new Point(19.45, 34.1)
                            );
                        }
                        break;
                    case THREE:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(50, 30),
                                    new Point(55, 38),
                                    new Point(28, 43),
                                    new Point(24.8, 91.5)
                            );
                        } else {
                            toAprilTags = new Bezier(90,
                                    spike.getEndPoint(),
                                    new Point(27, -0.5),
                                    new Point(28, 2),
                                    new Point(35, 22),
                                    new Point(29.5, 38.8)
                            );
                        }
                        break;
                    case FOUR:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new MergedBezier(
                                    new Bezier(-30,
                                            spike.getEndPoint(),
                                            new Point(42, -1),
                                            new Point(56, -10),
                                            new Point(46, -60),
                                            new Point(32, -65)
                                    ),
                                    new Bezier(-70,
                                            new Point(32, -65),
                                            new Point(36.11, -88.3)
                                    )
                            );
                        } else {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(20, 3.7),
                                    new Point(42, -22),
                                    new Point(33, -34)
                            );
                        }
                        break;
                    case FIVE:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new MergedBezier(
                                    new Bezier(-30,
                                            spike.getEndPoint(),
                                            new Point(46.7, -1),
                                            new Point(56, -10),
                                            new Point(46, -60),
                                            new Point(32, -65)
                                    ),
                                    new Bezier(-65,
                                            new Point(32, -65),
                                            new Point(29.9, -90)
                                    )
                            );
                        } else {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(32, -18),
                                    new Point(45, -18),
                                    new Point(30.4, -35.9) //TODO: toast cclara
                            );
                        }
                        break;
                    case SIX:
                        if (startPos == StartPos.OUT) {
                            toAprilTags = new MergedBezier(
                                    new Bezier(-30,
                                            spike.getEndPoint(),
                                        new Point(37, 22.5),
                                        new Point(56, 5),
                                        new Point(46, -60),
                                        new Point(32, -65)
                                    ),
                                    new Bezier(-65,
                                    new Point(32, -65),
                                    new Point(34, -89.2)
                                )
                            );
                        } else {
                            toAprilTags = new Bezier(-90,
                                    spike.getEndPoint(),
                                    new Point(20.5, -17),
                                    new Point(20.5, -34.3)
                            );
                        }
                        break;
                }

                bruh = new Bezier(mult * 90, toAprilTags.getEndPoint());

                if (startPos == StartPos.IN && !red) {
                    park = new Bezier(0, toAprilTags.getEndPoint(), new Point(1, 35.5));
                } else if (startPos == StartPos.IN) {
                    park = new Bezier(0, toAprilTags.getEndPoint(), new Point(7, -37.5));
                } else {
                    park = new Bezier(mult * 90, toAprilTags.getEndPoint());

                }

//                park = new Bezier(
//                        new Point(0, 0),
//                        new Point(mult * 95, 0)
//                );
            }
        }

        SequentialCommand scheduler = new SequentialCommand(
                // spike
                new FollowTrajectory(motionPlanner, spike),
                new Wait(300),
                new Spintake(false, false),
                new Wait(400),

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
                new FollowTrajectory(motionPlanner, bruh),
                new Wait(200),
                new PosOuttakePixel(),
                new Wait(200),
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

                telemetry.addData("Current", Gnocchi.slides.getCurrent1() +
                        Gnocchi.slides.getCurrent2() + drive.totalCurrent());
                telemetry.addData("", motionPlanner.getTelemetry());
                telemetry.update();
            }

            Gnocchi.webcam.stopStreaming();
    }
}