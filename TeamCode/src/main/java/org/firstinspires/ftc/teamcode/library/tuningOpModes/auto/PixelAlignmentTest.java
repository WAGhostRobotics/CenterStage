package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.component.Webcam;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.opencv.core.Point;


@Autonomous
public class PixelAlignmentTest extends LinearOpMode {
    Webcam webcam = new Webcam();
    Point center;
    public static double p = 0.01;
    public static double i = 0;
    public static double d = 0;
    public static double KStatic = 0.5;
    public static double KStaticTurn = 0.3;
    PIDController YControl = new PIDController(p, i, d);
    PIDController headingControl = new PIDController(p, i, d);
    double CAMERA_TARGET_X = 600;
    double targetHeading;
    double yError;
    double xError;
    double headingError;
    double yPower;
    final double PERMISSIBLE_ERROR = 2;
    final double PERMISSIBLE_ERROR_TURN = 2;
    double theta = 0;
    double driveTurn;
    Drivetrain drive;
    Localizer localizer;
    final double movementPower = 0.8;
    @Override
    public void runOpMode() {
        webcam.init(hardwareMap);
        waitForStart();
        YControl.setIntegrationBounds(-10000000, 10000000);
        headingControl.setIntegrationBounds(-10000000, 10000000);
        targetHeading = 0;
        drive = new MecanumDrive(hardwareMap);
        while (opModeIsActive()) {
            double heading = Math.toDegrees(localizer.getHeadingImu());
            headingError = targetHeading - heading;
            driveTurn = headingControl.calculate(0, headingError);
            driveTurn += Math.signum(driveTurn)*KStaticTurn;
            driveTurn = (headingError>=PERMISSIBLE_ERROR_TURN) ? 0 : driveTurn;
            center = webcam.getPixelLocation();
            double currentX = center.x;
            yError = currentX - CAMERA_TARGET_X;
            yPower = YControl.calculate(0, yError);
            yPower += Math.signum(yPower) * KStatic;
            yPower = (Math.abs(yError)>=PERMISSIBLE_ERROR) ? yPower : 0;
            theta = (yError>0) ? 90 : -90;
            drive.drive(yPower, theta, driveTurn, movementPower);
            telemetry.addData("Position X", center.x);
            telemetry.addData("Position Y", center.y);
            telemetry.update();
        }
    }
}
