/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CommandBase.Climb;
import org.firstinspires.ftc.teamcode.CommandBase.CollectPixel;
import org.firstinspires.ftc.teamcode.CommandBase.OuttakePixel;
import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.DriveStyle;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.DriverOrientedControl;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;


public class TeleOpParent extends LinearOpMode {



    DriveStyle.DriveType type = DriveStyle.DriveType.MECANUMARCADE;

    public double power = 0.8;

    CollectPixel collectPixel;
    Climb climb;
    OuttakePixel outtakePixel;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
        * Gamepad Stuff
        * gamepad2 -> driving
        * gamepad1 -> everything else
        * */

        GamepadEx driverOp = new GamepadEx(gamepad2);
        ToggleButtonReader rightStickReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        GamepadEx funcOp = new GamepadEx(gamepad1);
        ToggleButtonReader rightStickReader1 = new ToggleButtonReader(
                funcOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        Gnocchi.init(hardwareMap);
//        if(Gnocchi.imu==null){
//            Gnocchi.initIMU();
//        }

        waitForStart();

        Drivetrain drive = new MecanumDrive(hardwareMap);

        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);

        WonkyDrive wonkyDrive = new WonkyDrive(this, hardwareMap, localizer, drive);

        collectPixel = new CollectPixel();
        climb = new Climb();
        outtakePixel = new OuttakePixel();

        Gnocchi.slides.setTargetPosition(Slides.TurnValue.RETRACTED.getTicks());

        while (opModeIsActive()) {

            //DRIVETRAIN STUFF
//            if (type == DriveStyle.DriveType.MECANUMARCADE){
//                drive.driveRobotCentric(
//                        power * driverOp.getLeftX(),
//                        power * driverOp.getLeftY(),
//                        power * driverOp.getRightX(),
//                        false
//                );
//            } else if (type == DriveStyle.DriveType.DRIVERORIENTED){
//                drive.driveFieldCentric(
//                        power * (Math.pow(driverOp.getLeftX(), 3)),
//                        power * (Math.pow(driverOp.getLeftY(), 3)),
//                        turningMultiplier * power * (Math.pow(driverOp.getRightX(), 3)),
//                        Gnocchi.imu.getCurrentHeading(),   // gyro value passed in here must be in degrees
//                        false
//                );
//            }

            wonkyDrive.drive(gamepad2, power);

            //re-initializes imu to correct heading if teleop starts at the wrong heading
            if (gamepad2.left_stick_button){
                localizer.initImu();
            }

            // slow-mo
            if (rightStickReader.getState()) {
                power = 0.2;
            } else {
                power = 0.65;
            }

            // SLIDES
            if (gamepad1.left_bumper) {
                collectPixel.stop();
                outtakePixel.stop();
                Gnocchi.slides.setTargetPosition(Gnocchi.slides.getTicks() + 50);
            } else if (gamepad1.right_bumper) {
                collectPixel.stop();
                outtakePixel.stop();
                Gnocchi.slides.setTargetPosition(Gnocchi.slides.getTicks() - 50);
            }
            Gnocchi.slides.update();

            // OUTTAKE
            if (gamepad1.a) {
                collectPixel.stop();
                outtakePixel.init();
            }

            if (gamepad1.b) {
                collectPixel.stop();
                Gnocchi.mainSail.out();
            } if (gamepad1.x) {
              continue; // so collectPixel can keep running
            } else {
                Gnocchi.mainSail.stop();
            }

            if (gamepad1.dpad_left) {
                Gnocchi.mainSail.adjustArm(true);
            } else if (gamepad1.dpad_right) {
                Gnocchi.mainSail.adjustArm(false);
            }

            outtakePixel.update();

            // INTAKE
            if (gamepad1.x) {
                outtakePixel.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.GROUND);
                collectPixel.init();
            } else if (gamepad1.b) {
                continue; // so it doesn't come in conflict with outtake
            } else {
                Gnocchi.intake.stop();
                Gnocchi.mainSail.stop();
            }
            // necessary? probably not. idk why im putting this here
            if (gamepad1.dpad_up) {
                Gnocchi.intake.adjustHeight(true);
            } else if (gamepad1.dpad_down) {
                Gnocchi.intake.adjustHeight(false);
            }

            collectPixel.update();

//            // AIRPLANE (haha DNE)
//            if (rightStickReader1.wasJustReleased()) {
//                Gnocchi.launcher.launch();
//            }

            // CLIMB
            if (gamepad1.right_stick_button) {
                outtakePixel.stop();
                collectPixel.stop();
                climb.init();
            }

            climb.update();
        }
    }
}