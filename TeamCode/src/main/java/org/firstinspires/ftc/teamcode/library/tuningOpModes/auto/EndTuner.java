package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Point;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

@Config
@Autonomous(name = "End Tuner", group = "tuning")
public class EndTuner extends LinearOpMode {

    public static double p = MotionPlanner.translationalControlEndX.getP(), i = MotionPlanner.translationalControlEndX.getI(), d = MotionPlanner.translationalControlEndX.getD();

    @Override
    public void runOpMode() throws InterruptedException {

        boolean stop = true;
        ElapsedTime wait = new ElapsedTime();

        TwoWheelLocalizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        boolean forward = true;
        motionPlanner.startTrajectory(new Bezier(new Point(0, 0), new Point(24, 0)));

        double timeout = 3;
        ElapsedTime timer = new ElapsedTime(0);

        while (!isStopRequested()) {

            motionPlanner.translationalControlEndX.setPID(p, i, d);
            motionPlanner.translationalControlEndY.setPID(p, i, d);
            motionPlanner.update();

            if(motionPlanner.isFinished()||timer.seconds()>timeout){

                if(stop){
                    timer.reset();
                    stop = false;
                }

                if(timer.seconds()>1){
                    stop = true;
                    timer.reset();
                    if(forward){
                        forward = false;
                        motionPlanner.startTrajectory(new Bezier(new Point(24, 0), new Point(0, 0)));
                    }else{
                        forward = true;
                        motionPlanner.startTrajectory(new Bezier(new Point(0, 0), new Point(24, 0)));
                    }
                }
            }else{
                stop = true;
            }

            telemetry.addData("X - error", motionPlanner.getSpline().getEndPoint().getX() - localizer.getX());
            telemetry.addData("Y - error", motionPlanner.getSpline().getEndPoint().getY() - localizer.getY());
            telemetry.addData("LowerBound", -1);
            telemetry.addData("UpperBound", 1);
            telemetry.addData("X", localizer.getX());
            telemetry.addData("Heading Error", motionPlanner.getHeadingError());
            telemetry.addData("Loop Speed", 1.0 / wait.seconds());
            telemetry.addData("End", motionPlanner.getSpline().getEndPoint().getX() + " " + motionPlanner.getSpline().getEndPoint().getY());
            telemetry.addLine(motionPlanner.getTelemetry());
            telemetry.update();

            wait.reset();
        }

    }
}