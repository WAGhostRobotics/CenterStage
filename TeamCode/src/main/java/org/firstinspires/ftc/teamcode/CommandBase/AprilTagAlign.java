package org.firstinspires.ftc.teamcode.CommandBase;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.auto.AutoParent;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;
import org.firstinspires.ftc.teamcode.library.vision.AprilTagDetect;
import org.openftc.apriltag.AprilTagDetection;

@Config
public class AprilTagAlign extends Command {

    // These axes are declared relative to the AprilTag pose
    private double u_t;
    private double xTranslation;
    // private double yTranslation;
    private MotionPlanner motionPlanner;
    private Localizer localizer;
    private AprilTagDetect pipe;
    private int tagID;
    private AprilTagDetection targetDetection = new AprilTagDetection();

    private final double INCHES_PER_METER = 39.3701;
    private final double PERMISSIBLE_ERROR = 1;
    public static double kStatic = 4;
    public static double kP = 0.08;

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
        if (targetDetection == null || targetDetection.center == null) {
            motionPlanner.startTrajectory(new Bezier(Math.toDegrees(localizer.getHeadingImu()),
                    new Point(localizer.getX(), localizer.getY()),
                    // y + x is intentional; the localizer and AprilTag axes are reversed
                    new Point(localizer.getX()+2, localizer.getY())
            ));
        }
        else {
            // Align horizontally only to face the AprilTag straight on
            u_t = (targetDetection.center.x - 640)  / 640 * 100 * kP;
            xTranslation = u_t + Math.signum(u_t) * kStatic;
            // yTranslation = targetDetection.pose.y * INCHES_PER_METER;
            motionPlanner.startTrajectory(new Bezier(Math.toDegrees(localizer.getHeadingImu()),
                    new Point(localizer.getX(), localizer.getY()),
                    // y + x is intentional; the localizer and AprilTag axes are reversed
                    new Point(localizer.getX(), localizer.getY()-xTranslation)
            ));
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(u_t) <= PERMISSIBLE_ERROR);
    }

    public double getXTranslation() {
        return xTranslation;
    }

    public String getTelemetry() {
        return "u(t): " + u_t + "\n" +
                "xTranslation: " + xTranslation + "\n" +
                "target pos: " + (localizer.getY()-xTranslation) + "\n" +
                "current pos: " + localizer.getY();
    }
}
