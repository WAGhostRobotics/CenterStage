package org.firstinspires.ftc.teamcode.teleop;

//import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive.Swerve;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

//@Photon
@TeleOp(group = "drive")
public class CustomLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        TwoWheelLocalizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        Drivetrain drive = new MecanumDrive(hardwareMap);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (!isStopRequested()) {
            timer.reset();

            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
//            drive.drive(Math.hypot(driveX, driveY), Math.toDegrees(Math.atan2(driveY, driveX)), driveTurn, 0.5);
//            drive.driveTest(0.5);

            drive.drive(Math.hypot(driveX, driveY), Math.toDegrees(Math.atan2(driveY, driveX)) - localizer.getHeadingImu(), driveTurn, 0.8 );
            localizer.update();

            telemetry.addData("x", localizer.getX());
            telemetry.addData("y", localizer.getY());
            telemetry.addData("heading", localizer.getHeading(Localizer.Angle.DEGREES));
            telemetry.addData("rawx", localizer.getRawX());
            telemetry.addData("rawy", localizer.getRawY());
            telemetry.addData("hz", 1.0/timer.seconds());
            telemetry.addData("Power", drive.getTelemetry());
            telemetry.update();
        }
    }
}
