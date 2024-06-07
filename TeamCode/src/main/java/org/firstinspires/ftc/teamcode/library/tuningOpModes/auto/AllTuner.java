package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;


@Config
@Autonomous(name = "All Tuner", group = "tuning")
public class AllTuner extends LinearOpMode {

    public static double pT = 0.03;
    public static double pX = 0.02;
    public static double pY = 0.01;
    public static double iT = 0;
    public static double iX = 0.00001;
    public static double iY = 0;
    public static double dT = 0;
    public static double dX = 0;
    public static double dY = 0;
    public static double KsT = 0.08;
    public static double KsX = 0.2;
    public static double KsY = 0.5;
    public PIDController headingControl= new PIDController(pT, iT, dT);
    public PIDController XControl= new PIDController(pX, iX, dX);
    public PIDController YControl= new PIDController(pY, iY, dY);
    public static double targetHeading = 0;
    public static double targetX = 0;
    public static double targetY = 0;
    public double currentHeading = 0;
    public double x;
    public double xError;
    public double y;
    public double yError;
    double magnitude;
    double theta;
    double driveTurn = 0;

    double xPower = 0;
    double yPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(this, hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        headingControl.setIntegrationBounds(-10000000, 10000000);
        XControl.setIntegrationBounds(-10000000, 10000000);
        YControl.setIntegrationBounds(-10000000, 10000000);

        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
            x = localizer.getX();
            y = localizer.getY();
            xError = targetX-x;
            yError = targetY-y;
            targetHeading = normalizeDegrees(targetHeading);
            currentHeading = normalizeDegrees(Math.toDegrees(localizer.getHeading()));

            headingControl.setPID(pT, iT, dT);
            XControl.setPID(pX, iX, dX);
            YControl.setPID(pY, iY, dY);
            double headingError = getHeadingError();
            double translationalError = Math.hypot(xError, yError);
            driveTurn = headingControl.calculate(0, headingError);
            driveTurn += Math.signum(headingError)*KsT;
            theta = normalizeDegrees(Math.toDegrees(Math.atan2(yError, xError)) - currentHeading);
            xError = Math.cos(Math.toRadians(theta))*translationalError;
            yError = Math.sin(Math.toRadians(theta))*translationalError;
            xPower = XControl.calculate(0, xError);
            yPower = YControl.calculate(0, yError);

            xPower = (Math.abs(xError)>2) ? xPower + Math.signum(xPower)* KsX : 0;
            yPower = (Math.abs(yError)>2) ? yPower + Math.signum(yPower)* KsY : 0;
            magnitude = Math.hypot(xPower, yPower);
            if (Math.abs(headingError)<2.0) {
                driveTurn = 0;
            }
            if (Math.abs(Math.hypot(xError, yError))<=2.0) {
                magnitude = 0;
            }
            drive.drive(magnitude, theta, driveTurn, .85);
            telemetry.addData("Current Heading: ", currentHeading);
            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.addData("Heading Error: ", getHeadingError());
            telemetry.addData("xError: ", xError);
            telemetry.addData("yError: ", yError);
            telemetry.addData("X: " , x);
            telemetry.addData("Y: ", y);
            telemetry.addData("xPower: ", xPower);
            telemetry.addData("yPower: ", yPower);
            telemetry.addData("Magnitude: ", magnitude);
            telemetry.addData("Theta: ", theta);
            telemetry.addData("DriveTurn: ", driveTurn);
            telemetry.addData("Total Error: ", Math.hypot(xError, yError));
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
