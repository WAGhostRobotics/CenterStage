package org.firstinspires.ftc.teamcode.library.tuningOpModes.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;
import org.firstinspires.ftc.teamcode.library.autoDrive.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.library.teleopDrive.WonkyDrive;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    public double lastX;
    public double lastY;
    public double x;
    public double y;
    public double velocity;
    public Drivetrain drive;
    public double power = 1;
    ElapsedTime timer;
    double [] velocities;
    double avgVel = 0;
    int length;
    int index = 0;
    double maxAvg;
    double angularV;
    double maxAcc;
    double maxAngV;
    double acc;

    double prevAngV;
    double angAcc;
    double maxDecc = 0;
    boolean braking;
    double brakeHeading = 0;
    double magnitude;
    double driveTurn;
    @Override
    public void runOpMode() throws InterruptedException {
        double lastV = 0;
        maxAvg = 0;
        timer = new ElapsedTime();
        length = 10;
        index = 0;
        velocities = new double[length];
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Localizer localizer = new TwoWheelLocalizer(this, hardwareMap);

        drive = new MecanumDrive(hardwareMap);

        double heading = 0;

        waitForStart();
//        timer.startTime();
        while (opModeIsActive()) {
//            index = 0;
            if (gamepad2.left_stick_button){
                localizer.initImu();
            }

            driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            magnitude = Math.hypot(driveX, driveY);
            if (magnitude==0) {
                if (!braking) {
                    braking = true;
                    brakeHeading = Math.toDegrees(localizer.getHeading());
                }
                drive.brake(localizer, driveTurn, .85);
                telemetry.addData("", drive.getTelemetry());
            }
            else {
                braking = false;
                drive.drive(magnitude, Math.toDegrees(Math.atan2(driveY, driveX)), driveTurn, .85);
            }
            heading = normalizeDegrees(localizer.getHeading(Localizer.Angle.DEGREES));
            prevAngV = angularV;
            angularV = localizer.getAngularVelocityImu();
            angAcc = (angularV-prevAngV)/timer.seconds();
            x = localizer.getX();
            y = localizer.getY();
            velocity = localizer.getRawVelocity();
            avgVel = localizer.getAvgVelocity();

            acc = localizer.getAcc();
            if (Math.abs(avgVel) > maxAvg) {
                maxAvg = Math.abs(avgVel);
            }
            if (acc > maxAcc) {
                maxAcc = Math.abs(acc);
            }
            if (acc < maxDecc) {
                maxDecc = acc;
            }
            if (Math.abs(angularV) > maxAngV) {
                maxAngV = Math.abs(angularV);
            }
            double acc2 = (lastV-avgVel)/timer.seconds();
            lastV = avgVel;
            lastX = x;
            lastY = y;

            telemetry.addData("Avg Velocity ", avgVel);
            telemetry.addData("Theta ", localizer.getTheta());
            telemetry.addData("Acc ", acc);
            telemetry.addData("Acc2 ", acc2);
            telemetry.addData("AngV ", angularV);
            telemetry.addData("AngAcc ", angAcc);
            telemetry.addData("heading ", heading);
            telemetry.addData("max angular v ", maxAngV);
            telemetry.addData("Gamepad Magnitute: ", Math.hypot(driveX, driveY));
            telemetry.addData("Gamepad Theta: ", Math.toDegrees(Math.atan2(driveY, driveX)));
            telemetry.addData("Velocity ", velocity);
            telemetry.addData("Velocities: ", velocities);
            telemetry.addData("Index: ", index);
            telemetry.addData("X ", x);
            telemetry.addData("Y ", y);
            telemetry.addData("Last X ", lastX);
            telemetry.addData("Last Y ", lastY);
            telemetry.addData("Timer ", timer.seconds());
            telemetry.addData("Index ", index);
            telemetry.addData("MaxV: ", maxAvg);
            telemetry.addData("MaxAcc: ", maxAcc);
            telemetry.addData("MaxDecc: ", maxDecc);
            telemetry.addData("Brake Heading: ", brakeHeading);
            telemetry.addData("Mag: ", magnitude);
            telemetry.addData("DriveTurn: ", driveTurn);
            telemetry.addData("Braking?: ", braking);
            timer.reset();
            telemetry.update();
            localizer.update();
        }

    }
}
