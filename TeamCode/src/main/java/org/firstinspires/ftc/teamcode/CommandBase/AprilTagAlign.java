package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.auto.AutoParent;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;
import org.firstinspires.ftc.teamcode.library.vision.AprilTagDetect;
import org.openftc.apriltag.AprilTagDetection;

public class AprilTagAlign extends Command {

    // These axes are declared relative to the AprilTag pose
    private double xTranslation;
    // private double yTranslation;
    private MotionPlanner motionPlanner;
    private Localizer localizer;
    private AprilTagDetect pipe;
    private int tagID;
    private AprilTagDetection targetDetection = new AprilTagDetection();

    private final double INCHES_PER_METER = 39.3701;
    private final double PERMISSIBLE_ERROR = 0.25;

    public AprilTagAlign(MotionPlanner motionPlanner, Localizer localizer, AprilTagDetect pipe, int tagID) {
        this.motionPlanner = motionPlanner;
        this.localizer = localizer;
        this.pipe = pipe;
        this.tagID = tagID;
    }

    @Override
    public void update() {
        for (AprilTagDetection detection: pipe.getLatestDetections()) {
            if (detection.id == tagID) {
                targetDetection = detection;
                break;
            }
        }
        // Align horizontally only to face the AprilTag straight on
        xTranslation = targetDetection.pose.x * INCHES_PER_METER;
        // yTranslation = targetDetection.pose.y * INCHES_PER_METER;
        motionPlanner.startTrajectory(new Bezier(
                new Point(localizer.getX(), localizer.getY()),
                // y + x is intentional; the localizer and AprilTag axes are reversed
                new Point(localizer.getX(), localizer.getY()+xTranslation)
        ));
    }

    @Override
    public boolean isFinished() {
        return (xTranslation <= PERMISSIBLE_ERROR);
    }
}
