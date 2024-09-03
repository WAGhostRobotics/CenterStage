package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.component.Imu;
import org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive.MecanumDrive;

@TeleOp (name="DistTest")
public class DistanceSensorTest extends LinearOpMode {
    AnalogInput distanceSensor;
    final double voltageInput = 3.3;
    double distVoltage;
    double distance;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        distanceSensor = hardwareMap.get(AnalogInput.class, "LeftDist");

        waitForStart();
        while (opModeIsActive()) {
            distVoltage = distanceSensor.getVoltage();
            distance = ((distVoltage / (voltageInput/1024)) * 6) - 300;
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            drive.drive(Math.hypot(driveX, driveY), Math.toDegrees(Math.atan2(driveY, driveX)), driveTurn, 0.9);
            telemetry.addData("Voltage: ", distVoltage);
            telemetry.addData("Distance: ", distance);
            telemetry.update();
        }
    }
}
