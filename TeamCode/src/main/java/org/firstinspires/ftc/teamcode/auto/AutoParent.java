package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.FollowTrajectory;
import org.firstinspires.ftc.teamcode.CommandBase.MainSailMove;
import org.firstinspires.ftc.teamcode.CommandBase.PosOuttakePixel;
import org.firstinspires.ftc.teamcode.CommandBase.PixelHolderFunc;
import org.firstinspires.ftc.teamcode.CommandBase.SlidesMove;
import org.firstinspires.ftc.teamcode.CommandBase.Wait;
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

public class AutoParent extends LinearOpMode {

    public enum StartPos {
        IN,
        OUT
    }

    boolean red = false;
    int mult;
    StartPos startPos = StartPos.IN;

    // vision stuff
    SpikeDetect.Location location;
    Webcam.AprilTagLocation aprilTagLocation = red ? Webcam.AprilTagLocation.SIX : Webcam.AprilTagLocation.THREE;

    Bezier spike;
    Bezier toAprilTags;
    Bezier park;

    /* command stuff
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

        Gnocchi.init(hardwareMap, red);

        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

        Gnocchi.webcam.init(hardwareMap);

        spike = new Bezier(
                new Point(1, 0)
        );

        if (red) {
            mult = -1;
        } else {
            mult = 1;
        }

        while (opModeInInit()) {

            //Gnocchi.webcam.scanForLocation(); // doesn't work D:
            location = Gnocchi.webcam.getLocation();

            if (location != null) {
                telemetry.addData("Location: ", location);
                telemetry.update();

                switch (location) {
                    case RIGHT:
                        if (!red && startPos == StartPos.IN) {
                            spike = new Bezier(
                                    -30,
                                    new Point(0, 0),
                                    new Point(10,-10),
                                    new Point(21, -11.5)
                            );
                        } else if (red && startPos == StartPos.OUT) {
                            spike = new Bezier(
                                    -30,
                                    new Point(0, 0),
                                    new Point(10,-13),
                                    new Point(21, -5)
                            );
                        } else {
                            spike = new Bezier(-120,
                                    new Point(0, 0),
                                    new Point(36, -10));
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.SIX : Webcam.AprilTagLocation.THREE;
                        break;
                    case MID:
                        if (!red && startPos == StartPos.IN) {
                            spike = new Bezier(mult*-170,
                                    new Point(0, 0),
                                    new Point(42, 2.5));

                        } else {
                            spike = new Bezier(mult*-170,
                                    new Point(0, 0),
                                    new Point(44, -4));
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.FIVE : Webcam.AprilTagLocation.TWO;
                        break;
                    case LEFT:
                        if (!red && startPos == StartPos.OUT || red && startPos == StartPos.IN) {
                            spike = new Bezier(
                                    30,
                                    new Point(0, 0),
                                    new Point(10, 10),
                                    new Point(22, 0)
                            );
                        } else {
                            if (red) {
                                spike = new Bezier(mult * 150,
                                        new Point(0, 0),
                                        new Point(37, 10.5));
                            } else {
                                spike = new Bezier(mult*150,
                                        new Point(0, 0),
                                        new Point(37, 2.5));

                            }
                        }
                        aprilTagLocation = red ? Webcam.AprilTagLocation.FOUR : Webcam.AprilTagLocation.ONE;
                        break;
                }
            }

            switch (aprilTagLocation) {
                case ONE:
                    if (startPos == StartPos.OUT) {
                        toAprilTags = new Bezier(90,
                                spike.getEndPoint(),
                                new Point(25, -20),
                                new Point(58, 30),
                                new Point(50, 58),
                                new Point(16, 80)
                        );
                    } else {
                        toAprilTags = new Bezier(90,
                                spike.getEndPoint(),
                                new Point(50, 20),
                                new Point(25, 25),
                                new Point(15, 37)
                        );
                    }
                    break;
                case TWO:
                    if (startPos == StartPos.OUT) {
                        toAprilTags = new Bezier(90,
                                spike.getEndPoint(),
                                new Point(45, 30),
                                new Point(30, 48),
                                new Point(16.2, 79)
                        );
                    } else {
                        toAprilTags = new Bezier(90,
                                spike.getEndPoint(),
                                new Point(40, 20),
                                new Point(25, 25),
                                new Point(18, 36)
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
                                new Point(17, -4),
                                new Point(20, 15),
                                new Point(27, 37)
                        );
                    }
                    break;
                case FOUR:
                    if (startPos == StartPos.OUT) {
                        toAprilTags = new Bezier(-90,
                                spike.getEndPoint(),
                                new Point(60, -30),
                                new Point(50, -60),
                                new Point(33, -88.5)
                        );
                    } else {

                    }
                    break;
                case FIVE:
                    if (startPos == StartPos.OUT) {
                        toAprilTags = new Bezier(-90,
                                spike.getEndPoint(),
                                new Point(60, -20),
                                new Point(38, -80),
                                new Point(31, -88.5)
                        );
                    } else {

                    }
                    break;
                case SIX:
                    if (startPos == StartPos.OUT) {
                        toAprilTags = new Bezier(-90,
                                spike.getEndPoint(),
                                new Point(35, 20),
                                new Point(60, -20),
                                new Point(32, -60),
                                new Point(27, -88.5)
                        );
                    } else {
                        toAprilTags = new Bezier();
                    }
                    break;
            }
        }

        SequentialCommand scheduler = new SequentialCommand(
                // spike
                new ParallelCommand(
                        new FollowTrajectory(motionPlanner, spike),
                        new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.SPIKE.getPosition())),
                        new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.SPIKE.getPosition()))
                ),
                new Wait(100),
                new PixelHolderFunc(false, false),
                new Wait(500),

                // april tag
                new ParallelCommand(
                        new SequentialCommand(
                                new Wait(700),
                                new MainSailMove(MainSail.ArmPos.RETRACT.getPosition(), MainSail.HolderPos.RETRACT.getPosition())
                        ),
                        new SequentialCommand(
                                new Wait(800),
                                new FollowTrajectory(motionPlanner, toAprilTags)
                        ),
                        new PixelHolderFunc(false, true)
                ),
                new Wait(700),
                new PosOuttakePixel(),
                new Wait(500),
                new PixelHolderFunc(false, false),
                new MainSailMove(MainSail.ArmPos.RETRACT.getPosition(), MainSail.HolderPos.RETRACT.getPosition()),
                new SlidesMove(Slides.TurnValue.INTAKE.getTicks())
//                new FollowTrajectory(motionPlanner, park)
        );

        scheduler.init();
        motionPlanner.startTrajectory(spike);

        while(opModeIsActive() && !isStopRequested()) {
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