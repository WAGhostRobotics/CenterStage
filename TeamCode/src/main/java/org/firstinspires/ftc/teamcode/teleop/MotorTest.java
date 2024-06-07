package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Motor test")
public class MotorTest extends LinearOpMode {
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            double driveY = Math.pow(-gamepad2.left_stick_x, 3);
            double driveX = Math.pow(-gamepad2.left_stick_y, 3);
            double magnitude = Math.hypot(driveX, driveY);
            double theta = Math.atan2(driveY, driveX);
            double driveTurn = Math.pow(-gamepad2.right_stick_x, 3);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double frontLeftPow = magnitude * cos / max + driveTurn;
            double frontRightPow = magnitude * sin / max - driveTurn;
            double backLeftPow = magnitude * sin / max + driveTurn;
            double backRightPow = magnitude * cos / max - driveTurn;

            if (magnitude + Math.abs(driveTurn) > 1) {
                frontLeftPow /= magnitude + driveTurn;
                frontRightPow /= magnitude + driveTurn;
                backLeftPow /= magnitude + driveTurn;
                backRightPow /= magnitude + driveTurn;
            }
            frontLeft.setPower(frontLeftPow);
            frontRight.setPower(frontRightPow);
            backLeft.setPower(backLeftPow);
            backRight.setPower(backRightPow);
        }


//
//        while (opModeIsActive()) {
//            if (gamepad1.y) {
//                frontLeft.setPower(0.5);
//            } else if (gamepad1.a) {
//                frontLeft.setPower(-0.5);
//            }else {
//                motor1.setPower(0);
//            }
//        }
    }
}

