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
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;

@TeleOp(name = "sketchyaf")
public class SketchyAF extends LinearOpMode {

    public double power = 1;
    public boolean pos = false;
    public boolean slidesInPos = true;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Gamepad Stuff
         * gamepad2 -> driving
         * gamepad1 -> everything else
         */

        GamepadEx driverOp = new GamepadEx(gamepad2);
        ToggleButtonReader rightStickReader = new ToggleButtonReader(
                driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );

        Gnocchi.init(hardwareMap);

        IntakePixel intakePixel = new IntakePixel();

        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);
        Drivetrain drive = new MecanumDrive(hardwareMap);
        WonkyDrive wonk = new WonkyDrive(this, hardwareMap, localizer, drive);

        waitForStart();

        Gnocchi.slides.setTargetPosition(Slides.TurnValue.RETRACTED.getTicks());
        Gnocchi.intake.setHeight(Intake.IntakeHeight.GROUND);

        while (opModeIsActive()) {

            wonk.drive(gamepad2, power);

            //re-initializes imu to correct heading if teleop starts at the wrong heading
            if (gamepad2.left_stick_button){
                localizer.initImu();
            }

            // INTAKE/OUTTAKE
            if (gamepad1.a) {
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition());
            } else if (gamepad1.b) {
                intakePixel.stop();
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.PLACE.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.PLACE.getPosition());
            }

            if (gamepad1.x) {
                Gnocchi.mainSail.out();
            }

//            else if (!gamepad1.dpad_left && !gamepad1.dpad_right){
//                Gnocchi.mainSail.stop();
//            }


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
            else if (!gamepad1.x){
                Gnocchi.mainSail.stop();
                Gnocchi.intake.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.GROUND);
            }

            // SLIDES
            if (Gnocchi.mainSail.armAtIntake() && Math.abs(Gnocchi.slides.getTicks() - Slides.TurnValue.INTAKE.getTicks()) <= 300) {
                slidesInPos = true;
            } else {
                slidesInPos = false;
            }

            if (gamepad1.y) {
                // retract & get ready for intake
                intakePixel.init();
            }

            if (gamepad1.dpad_down) {
                intakePixel.stop();
                Gnocchi.slides.setTargetPosition(Slides.TurnValue.RETRACTED.getTicks());
                pos = true;
            }

            if (gamepad1.dpad_up) {
                intakePixel.stop();
                Gnocchi.slides.setTargetPosition(Slides.TurnValue.CLIMB.getTicks());
                pos = true;
            }

            if (gamepad1.right_bumper) {
                intakePixel.stop();
                pos = false;
                Gnocchi.slides.goUp();
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

//            if (gamepad1.right_trigger >= 0.01) {
//                Gnocchi.intake.adjustHeight(true);
//            } else if (gamepad1.left_trigger >= 0.01) {
//                Gnocchi.intake.adjustHeight(false);
//            }

            intakePixel.update();

            telemetry.addData("Arm", Gnocchi.mainSail.armAtIntake());
            telemetry.addData("Slides", Gnocchi.slides.getTicks());
            telemetry.update();
        }
    }
}