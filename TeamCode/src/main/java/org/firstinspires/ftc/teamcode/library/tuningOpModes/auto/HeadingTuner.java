package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.teamcode.core.Gnocchi.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;


@Config
@Autonomous(name = "Heading Tuner", group = "tuning")
public class HeadingTuner extends LinearOpMode {

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double Ks = 0;
    public PIDController headingControl= new PIDController(p, i, d);
    public static double targetHeading = 0;
    public double currentHeading = 0;
    double driveTurn = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(this, hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        headingControl.setIntegrationBounds(-10000000, 10000000);

        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
            targetHeading = normalizeDegrees(targetHeading);
            currentHeading = normalizeDegrees(Math.toDegrees(localizer.getHeading()));
            headingControl.setPID(p, i, d);
            double error = getHeadingError();
            driveTurn = headingControl.calculate(0, error);
            driveTurn += Math.signum(error)*Ks;
            if (Math.abs(error)<2) {
                drive.drive(0, 0, 0 ,0);
            }
            else
                drive.drive(0, 0, driveTurn, .85);
            telemetry.addData("Heading Error", getHeadingError());
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Current", currentHeading);
            telemetry.addData("Driveturn", driveTurn);
            telemetry.update();
        }

    }
    public double getHeadingError(){
        double headingError = targetHeading - currentHeading;

        if(headingError > 180){
            headingError -= 360;
        }else if(headingError<-180){
            headingError += 360;
        }

        return headingError;
    }


}
