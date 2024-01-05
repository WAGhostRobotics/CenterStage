/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.library.tuningOpModes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;


@Config
@TeleOp(name = "Wonk Tuner", group = "competition")
public class WonkTuner extends LinearOpMode {

    public double power = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);

        Drivetrain drive = new MecanumDrive(hardwareMap);

        waitForStart();

        WonkyDrive wonk = new WonkyDrive(this, hardwareMap, localizer, drive);

        while (opModeIsActive()) {

            //DRIVETRAIN STUFF
//            // re-initializes imu to correct heading if teleop starts at the wrong heading
            if (gamepad2.left_stick_button){
                localizer.initImu();
            }

            wonk.drive(gamepad2, power);

            telemetry.addData("Angle Error", wonk.getHeadingError());
            telemetry.addData("Strafe Accel", wonk.getStrafeVelo());
            telemetry.addData("Heading", wonk.getCurrentHeading());
            telemetry.addData("", wonk.getTelemetry());
            telemetry.addData("Last heading", wonk.getLastHeading());
            telemetry.update();
        }
    }
}