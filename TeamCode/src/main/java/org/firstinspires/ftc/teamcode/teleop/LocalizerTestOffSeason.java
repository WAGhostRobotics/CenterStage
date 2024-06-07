package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.autoDrive.HansenPKLocalizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

@TeleOp
public class LocalizerTestOffSeason extends LinearOpMode {
    Drivetrain drive;
    HansenPKLocalizer localizer;

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap);
        localizer = new HansenPKLocalizer(this, hardwareMap);
        while(opModeIsActive()) {
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            drive.drive(Math.hypot(driveX, driveY), Math.toDegrees(Math.atan2(driveY, driveX)) - localizer.getHeading(), driveTurn, 0.8 );
            localizer.update();
            telemetry.addData("", localizer.getTelemetry());
            telemetry.update();
        }

    }
}
