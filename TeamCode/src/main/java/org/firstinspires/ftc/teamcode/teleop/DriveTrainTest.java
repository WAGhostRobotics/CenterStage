package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.component.Imu;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

@TeleOp (name="DriveTrainTest")
public class DriveTrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Imu imu = new Imu(hardwareMap);
        imu.initImuThread(this);
        
        waitForStart();
        while (opModeIsActive()) {
            double heading = imu.getHeading();
            double angV = imu.getAngularVelocity();
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            drive.drive(Math.hypot(driveX, driveY), Math.toDegrees(Math.atan2(driveY, driveX)), driveTurn, 0.9);
            telemetry.addData("Heading: ", heading);
            telemetry.addData("AngV: ", angV);
            telemetry.update();
        }
    }
}
