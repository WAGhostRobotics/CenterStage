package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.MainSailMove;
import org.firstinspires.ftc.teamcode.CommandBase.OuttakePixel;
import org.firstinspires.ftc.teamcode.CommandBase.PixelHolderFunc;
import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.HeadingFollowerPath;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;
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

    HeadingFollowerPath board;
    HeadingFollowerPath park;

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

    SequentialCommand scheduler = new SequentialCommand(
            // move to position
            new ParallelCommand(
                    new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.SPIKE.getPosition())),
                    new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.SPIKE.getPosition()))
            ),
            new PixelHolderFunc(false, false),
            new MainSailMove(MainSail.ArmPos.RETRACT.getPosition(), MainSail.HolderPos.RETRACT.getPosition()),
            // move to position
            // use april tags
            new OuttakePixel(),
            new PixelHolderFunc(false, false)
            // park
    );


    @Override
    public void runOpMode() throws InterruptedException {
        Gnocchi.webcam.init(hardwareMap);
//        waitForStart();

        while (!isStarted() && !isStopRequested()) {
            Gnocchi.webcam.scanForLocation();
            location = Gnocchi.webcam.getLocation();

            if (location != null) {
                telemetry.addData("Location: ", location);
                telemetry.update();
            }

            switch (startPos) {
                case BLUE_IN:
                case BLUE_OUT:
                case RED_IN:
                case RED_OUT:
            }

        }

        while(opModeIsActive()) {
//            telemetry.addData("Contour Area: ", pipe.getContourArea());
//            telemetry.addData("X Center: ", pipe.getCenter().x);
//            telemetry.addData("Y Center: ", pipe.getCenter().y);
//            telemetry.update();
        }

        Gnocchi.webcam.stopStreaming();
    }
}