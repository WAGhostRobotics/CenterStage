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
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public boolean pos = false;
    public boolean slidesInPos = true;
    boolean out = false;
    double movementPwr = 0.85;
    ElapsedTime timer;

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
        ToggleButtonReader driveControllerY = new ToggleButtonReader(
                driverOp2, GamepadKeys.Button.Y
        );

        Gnocchi.init(hardwareMap, false, false);

        IntakePixel intakePixel = new IntakePixel();
        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        Gnocchi.slides.setTargetPosition(Slides.TurnValue.RETRACTED.getTicks());
        Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT);

        timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()) {

            if (timer.seconds() >= 85) {
                // LAUNCHER
                if (gamepad2.x) {
                    Gnocchi.launcher.launch();
                } else if (gamepad2.y) {
                    Gnocchi.launcher.close();
                }
            }

            // DRIVE
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            drive.drive(Math.hypot(driveX, driveY), Math.toDegrees(Math.atan2(driveY, driveX)), driveTurn, movementPwr);

            if (stick.wasJustReleased() && movementPwr == 1) {
                movementPwr = 0.5;
            } else if (stick.wasJustReleased() && movementPwr == 0.5){
                movementPwr = 1;
            }
            if (driveControllerY.wasJustReleased()) {
                localizer.initImu();
            }

//            //re-initializes imu to correct heading if teleop starts at the wrong heading
//            if (gamepad2.left_stick_button){
//                localizer.initImu();
//            }

            telemetry.addData("AHHH", out);


            // INTAKE/OUTTAKE

            // mainsail go in/out
            if (bReader.wasJustReleased() && out) {
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition());
                out = false;
            } else if (bReader.wasJustReleased() && !out) {
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.PLACE.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.PLACE.getPosition());
                out = true;
            }

            // outtake
            if (gamepad1.y) {
                Gnocchi.mainSail.out();
            }

            // intake from pixel holder if pixel is stuck
            if (gamepad2.dpad_up) {
                Gnocchi.mainSail.in();
            }

            // intake
            if (gamepad1.dpad_left && slidesInPos) {
                intakePixel.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.INTAKE);
//                Gnocchi.slides.setTargetPosition(Slides.TurnValue.SUPER_RETRACTED.getTicks());
                Gnocchi.intake.in();
                Gnocchi.mainSail.in();
            }
            else if (gamepad1.dpad_right && !gamepad1.y) {
                // spit pixel out
                intakePixel.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.INTAKE);
                Gnocchi.intake.out();
            }
            else if (!gamepad1.y && !gamepad1.dpad_left && !gamepad2.dpad_up && !gamepad1.right_stick_button && !gamepad1.left_stick_button){
                // stop intake
                Gnocchi.mainSail.stop();
                Gnocchi.intake.stop();
                Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT);
            }

            // adjust intake height
            if (gamepad1.right_stick_button) {
                Gnocchi.intake.up();
            } else if (gamepad1.left_stick_button) {
                Gnocchi.intake.down();
            }

            // adjust arm height
            if (gamepad1.right_trigger >= 0.01) {
                Gnocchi.mainSail.adjustArm(true);
            } else if (gamepad1.left_trigger >= 0.01) {
                Gnocchi.mainSail.adjustArm(false);
            }

            // SLIDES
            if (Gnocchi.mainSail.armAtIntake() && Math.abs(Gnocchi.slides.getTicks() - Slides.TurnValue.INTAKE.getTicks()) <= 300) {
                slidesInPos = true;
            } else {
                slidesInPos = false;
            }

            // retract & get ready for intake
            if (gamepad1.a) {
                pos = true;
                out = false;
                intakePixel.init();
            }

            // retract to climb
            if (gamepad1.dpad_down) {
                intakePixel.stop();
                Gnocchi.slides.setTargetPosition(Slides.TurnValue.RETRACTED.getTicks());
                pos = true;
            }

            // extend to climb
            if (gamepad1.dpad_up) {
                intakePixel.stop();
                out = true;
                Gnocchi.slides.setTargetPosition(Slides.TurnValue.CLIMB.getTicks());
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.MID.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition());
                pos = true;
            }

            // slides up
            if (gamepad1.right_bumper) {
                intakePixel.stop();
                pos = false;
                out = true;
                Gnocchi.slides.goUp();
                Gnocchi.mainSail.moveArm(MainSail.ArmPos.PLACE.getPosition());
                Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.PLACE.getPosition());
            } else if (gamepad1.left_bumper) {
                // slides down
                intakePixel.stop();
                pos = false;
                Gnocchi.slides.goDown();
            } else if (!pos) {
                // stop
                Gnocchi.slides.stopSlides();
            }

            if (pos) {
                Gnocchi.slides.update(); // only update if set to go to a position
            }


            intakePixel.update();
            bReader.readValue();
            localizer.update();
            telemetry.addData("Finished", intakePixel.isFinished());
            telemetry.addData("Slides", Gnocchi.slides.getTicks());
            telemetry.addData("X: ", localizer.getX());
            telemetry.addData("Y: ", localizer.getY());
            telemetry.addData("Heading: " , localizer.getHeading());
            telemetry.addData("RawX: ", localizer.getRawX());
            telemetry.addData("RawY: ", localizer.getRawY());
            telemetry.addData("Current Draw for spinTake: ", Gnocchi.intake.getMotorCurrentDraw());
            telemetry.addData("Drivetrain current: ", drive.totalCurrent());
            telemetry.addData("Drivetrain current: ", drive.totalCurrent());
            telemetry.update();
        }
    }
}