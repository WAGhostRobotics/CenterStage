package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
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
@Autonomous(name = "Move And Turn Tuner", group = "tuning")
public class MoveAndTurnTuner extends LinearOpMode {


    public static double p = MotionPlanner.headingControl.getP(), i = MotionPlanner.headingControl.getI(), d = MotionPlanner.headingControl.getD();


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
        motionPlanner.startTrajectory(new Bezier(90, new Point(0, 0), new Point(40, 0)));

        double timeout = 7;
        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {

            motionPlanner.headingControl.setPID(p, i, d);
            motionPlanner.update();

            if(motionPlanner.isFinished()||timer.seconds()>timeout){

                if(stop){
                    wait.reset();
                    stop = false;
                }



                if(wait.seconds()>3) {
                    stop = true;
                    timer.reset();
                    if (forward) {
                        forward = false;
                        motionPlanner.startTrajectory(new Bezier(0, new Point(40, 0), new Point(0, 0)));
                    } else {
                        forward = true;
                        motionPlanner.startTrajectory(new Bezier(90, new Point(0, 0), new Point(40, 0)));
                    }
                }
            }else{
                stop = true;
            }


            telemetry.addData("Heading Error", motionPlanner.getHeadingError());
            telemetry.addData("Target", 0);
            telemetry.addData("Finished", motionPlanner.isFinished());
            telemetry.addData("X - error", motionPlanner.getSpline().getEndPoint().getX() - localizer.getX());
            telemetry.addData("Y - error", motionPlanner.getSpline().getEndPoint().getY() - localizer.getY());
            telemetry.update();


        }

    }
}