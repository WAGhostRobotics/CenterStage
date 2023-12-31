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
@Autonomous(name = "Heading End Tuner", group = "tuning")
public class HeadingEndTuner extends LinearOpMode {


    public static double p = MotionPlanner.headingControlEnd.getP(), i = MotionPlanner.headingControlEnd.getI(), d = MotionPlanner.headingControlEnd.getD();

    public static double trackWidth = Localizer.LATERAL_DISTANCE;




    @Override
    public void runOpMode() throws InterruptedException {


        boolean stop = true;
        ElapsedTime wait = new ElapsedTime();


        TwoWheelLocalizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MotionPlanner motionPlanner = new MotionPlanner(drive, localizer, hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();


        double angle = 90;
        motionPlanner.startTrajectory(new Bezier(
                angle,
                new Point(0,0),
                new Point(0, 0)
        ));


        double timeout = 7;
        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {

            motionPlanner.headingControlEnd.setPID(p, i, d);
            Localizer.LATERAL_DISTANCE = trackWidth;
            motionPlanner.update();

            if(motionPlanner.isFinished()||timer.seconds()>timeout){

                if(stop){
                    wait.reset();
                    stop = false;
                }



                if(wait.seconds()>3) {
                    stop = true;
                    timer.reset();
                    angle += 90;
                    motionPlanner.startTrajectory(new Bezier(
                            angle,
                            new Point(0, 0),
                            new Point(0, 0)
                    ));
                }
            }else{
                stop = true;
            }


            telemetry.addData("Heading Error", motionPlanner.getHeadingError());
            telemetry.addData("Target", 0);
            telemetry.update();


        }

    }
}