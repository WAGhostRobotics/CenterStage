package org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.util.AnalogEncoder;

public class Swerve implements Drivetrain {

    private ModuleV2 frontLeft, frontRight, backRight, backLeft;

    private final double TRACK_WIDTH = 10;
    private final double WHEEL_BASE = 10;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public final double voltageConstant = 13.27;

    public Swerve(HardwareMap hwMap){

        DcMotor leftFront = hwMap.get(DcMotor.class, "lf");
        CRServo leftFrontPivot = hwMap.get(CRServo.class, "lfPivot");
        leftFrontPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder leftFrontEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "lfEnc"), 15.78, false);
        frontLeft = new ModuleV2(leftFront, leftFrontPivot, leftFrontEnc);

        DcMotor rightFront = hwMap.get(DcMotor.class, "rf");
        CRServo rightFrontPivot = hwMap.get(CRServo.class, "rfPivot");
        rightFrontPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder rightFrontEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "rfEnc"), -104.33, false);
        frontRight = new ModuleV2(rightFront, rightFrontPivot, rightFrontEnc);

        DcMotor leftRear = hwMap.get(DcMotor.class, "lr");
        CRServo leftRearPivot = hwMap.get(CRServo.class, "lrPivot");
        leftRearPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder leftRearEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "lrEnc"), -94.87, false);
        backLeft = new ModuleV2(leftRear, leftRearPivot, leftRearEnc);

        DcMotor rightRear = hwMap.get(DcMotor.class, "rr");
        CRServo rightRearPivot = hwMap.get(CRServo.class, "rrPivot");
        rightRearPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogEncoder rightRearEnc = new AnalogEncoder(hwMap.get(AnalogInput.class, "rrEnc"), 202.11, false);
        backRight = new ModuleV2(rightRear, rightRearPivot, rightRearEnc);

    }


    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower) {
        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        frontLeft.setPower(movementPower * frontLeftPower);
        frontRight.setPower(movementPower * frontRightPower);
        backLeft.setPower(movementPower * backLeftPower);
        backRight.setPower(movementPower * backRightPower);

    }


    public void driveCommon(double magnitude, double theta, double driveTurn){
        double x = magnitude * Math.cos(Math.toRadians(theta));
        double y = magnitude * Math.sin(Math.toRadians(theta));

        double v = driveTurn * (WHEEL_BASE / Math.hypot(WHEEL_BASE, TRACK_WIDTH));


        double frontY = y + v;
        double backY = y - v;

        double v1 = driveTurn * (TRACK_WIDTH / Math.hypot(WHEEL_BASE, TRACK_WIDTH));

        double leftX = x - v1;
        double rightX = x + v1;

        frontLeft.setTargetAngle(Math.toDegrees(Math.atan2(frontY, leftX)));
        frontRight.setTargetAngle(Math.toDegrees(Math.atan2(frontY, rightX)));
        backLeft.setTargetAngle(Math.toDegrees(Math.atan2(backY, leftX)));
        backRight.setTargetAngle(Math.toDegrees(Math.atan2(backY, rightX)));

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();

        frontLeftPower = Math.hypot(leftX, frontY);
        frontRightPower = Math.hypot(rightX, frontY);
        backRightPower = Math.hypot(rightX, backY);
        backLeftPower = Math.hypot(leftX, backY);

    }



    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower) {

        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(movementPower * frontLeftPower);
        frontRight.setPower(movementPower * frontRightPower);
        backLeft.setPower(movementPower * backLeftPower);
        backRight.setPower(movementPower * backRightPower);

    }


    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftPower *= movementPower;
        frontRightPower *= movementPower;
        backLeftPower *= movementPower;
        backRightPower *= movementPower;

        scaleByVoltage(voltage);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }


    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

        driveCommon(magnitude, theta, driveTurn);

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftPower *= movementPower;
        frontRightPower *= movementPower;
        backLeftPower *= movementPower;
        backRightPower *= movementPower;

        scaleByVoltage(voltage);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }

    public void driveTest(double magnitude, double power) {
        driveCommon(1, 0, 0);
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }


    public void scaleByVoltage(double voltage){
        frontLeftPower /= voltage;
        frontRightPower /= voltage;
        backLeftPower /= voltage;
        backRightPower /= voltage;


        frontLeftPower *= voltageConstant;
        frontRightPower *= voltageConstant;
        backLeftPower *= voltageConstant;
        backRightPower *= voltageConstant;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if(max>1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

    }

    @Override
    public String getTelemetry() {
        return "Front Left: " + frontLeft.getMotorPower() + "\n"
                + "Front Right: " + frontRight.getMotorPower() + "\n"
                + "Back Left: " + backLeft.getMotorPower() + "\n"
                + "Back Right: " + backRight.getMotorPower() + "\n"
                + "Motor mult:\n"
                + "FL" + frontLeft.getMotorMultiplier()
                + "\nFR" + frontRight.getMotorMultiplier()
                + "\nBL" + backLeft.getMotorMultiplier()
                + "\nBR" + backRight.getMotorMultiplier()
                + "\nTarget:\n"
                + "Front Left: " + frontLeft.getTargetAngle() + "\n"
                + "Front Right: " + frontRight.getTargetAngle() + "\n"
                + "Back Left: " + backLeft.getTargetAngle() + "\n"
                + "Back Right: " + backRight.getTargetAngle() + "\n"
                + "FL: " + frontLeft.getModuleAngle() + "\n"
                + "FR: " + frontRight.getModuleAngle() + "\n"
                + "BL: " + backLeft.getModuleAngle() + "\n"
                + "BR: " + backRight.getModuleAngle() + "\n"
                + "FL error: " + normalizeDegrees(frontLeft.getTargetAngle() - frontLeft.getModuleAngle()) + "\n"
                + "FR error: " + normalizeDegrees(frontRight.getTargetAngle() - frontRight.getModuleAngle()) + "\n"
                + "BL error: " + normalizeDegrees(backLeft.getTargetAngle() - backLeft.getModuleAngle()) + "\n"
                + "BR error: " + normalizeDegrees(backRight.getTargetAngle() - backRight.getModuleAngle());
    }
}
