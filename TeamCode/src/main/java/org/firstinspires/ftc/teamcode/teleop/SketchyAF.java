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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.IntakePixel;
import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

@TeleOp(name = "sketchyaf")
public class SketchyAF extends LinearOpMode {

    public boolean pos = false;
    public boolean slidesInPos = true;
    boolean out = false;
    double movementPwr = 0.95;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Gamepad Stuff
         * gamepad2 -> driving
         * gamepad1 -> everything else
         */

        GamepadEx driverOp = new GamepadEx(gamepad1);
        ToggleButtonReader bReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.B
        );

        GamepadEx driverOp2 = new GamepadEx(gamepad2);
        ToggleButtonReader stick = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        Gnocchi.init(hardwareMap, false);

        IntakePixel intakePixel = new IntakePixel();

//        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        Drivetrain drive = new MecanumDrive(hardwareMap);
//        WonkyDrive wonk = new WonkyDrive(this, hardwareMap, localizer, drive);

        waitForStart();

        Gnocchi.slides.setTargetPosition(Slides.TurnValue.RETRACTED.getTicks());
        Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT);

        while (opModeIsActive()) {

            // DRIVE
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            drive.drive(Math.hypot(driveX, driveY), Math.toDegrees(Math.atan2(driveY, driveX)), driveTurn, movementPwr);

            if (stick.wasJustReleased() && movementPwr == 0.95) {
                movementPwr = 0.5;
            } else if (stick.wasJustReleased() && movementPwr == 0.5){
                movementPwr = 0.95;
            }

//            wonk.drive(gamepad2, 1);

//            //re-initializes imu to correct heading if teleop starts at the wrong heading
//            if (gamepad2.left_stick_button){
//                localizer.initImu();
//            }

            telemetry.addData("AHHH", out);

            // INTAKE/OUTTAKE
            if (bReader.wasJustReleased() && out) {
                    Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition());
                    Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition());
                out = false;
            } else if (bReader.wasJustReleased() && !out) {
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.PLACE.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.PLACE.getPosition());
                out = true;
            }

            if (gamepad1.x) {
                Gnocchi.mainSail.out();
            }

            if (gamepad2.dpad_up) {
                Gnocchi.mainSail.in();
            }

            if (gamepad1.dpad_left && slidesInPos) {
                intakePixel.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.INTAKE);
                Gnocchi.intake.in();
                Gnocchi.mainSail.in();
            }
            else if (gamepad1.dpad_right && !gamepad1.x) {
                // spit pixel out
                intakePixel.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.INTAKE);
                Gnocchi.intake.out();
            }
            else if (!gamepad1.x && !gamepad1.dpad_left && !gamepad2.dpad_up && !gamepad1.right_stick_button && !gamepad1.left_stick_button){
                Gnocchi.mainSail.stop();
                Gnocchi.intake.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT);
            }

            if (gamepad1.right_stick_button) {
                Gnocchi.intake.up();
            } else if (gamepad1.left_stick_button) {
                Gnocchi.intake.down();
            }

            // SLIDES
            if (Gnocchi.mainSail.armAtIntake() && Math.abs(Gnocchi.slides.getTicks() - Slides.TurnValue.INTAKE.getTicks()) <= 300) {
                slidesInPos = true;
            } else {
                slidesInPos = false;
            }

            if (gamepad1.y) {
                // retract & get ready for intake
                pos = true;
                out = false;
                intakePixel.init();
            }

            if (gamepad1.dpad_down) {
                intakePixel.stop();
                Gnocchi.slides.setTargetPosition(Slides.TurnValue.RETRACTED.getTicks());
                pos = true;
            }

            if (gamepad1.dpad_up) {
                intakePixel.stop();
                out = true;
                Gnocchi.slides.setTargetPosition(Slides.TurnValue.CLIMB.getTicks());
                pos = true;
            }

            if (gamepad1.right_bumper) {
                intakePixel.stop();
                pos = false;
                out = true;
                Gnocchi.slides.goUp();
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.PLACE.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.PLACE.getPosition());
            } else if (gamepad1.left_bumper) {
                intakePixel.stop();
                pos = false;
                Gnocchi.slides.goDown();
            } else if (!pos) {
                Gnocchi.slides.stopArm();
            }

            if (pos) {
                Gnocchi.slides.update(); // only update if set to go to a position
            }

            if (gamepad1.right_trigger >= 0.01) {
                Gnocchi.mainSail.adjustArm(true);
            } else if (gamepad1.left_trigger >= 0.01) {
                Gnocchi.mainSail.adjustArm(false);
            }

            // LAUNCHER
            if (gamepad2.x) {
                Gnocchi.launcher.launch();
            } else if (gamepad2.y) {
                Gnocchi.launcher.close();
            }

            intakePixel.update();
            bReader.readValue();

            telemetry.addData("Finished", intakePixel.isFinished());
            telemetry.addData("Slides", Gnocchi.slides.getTicks());
            telemetry.update();
        }
    }
}