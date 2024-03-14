package org.firstinspires.ftc.teamcode.library.drivetrain.mecanumDrive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.library.drivetrain.Drivetrain;

@Config
public class MecanumDrive implements Drivetrain {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;

    private double sin;
    private double cos;
    private double maxMovement;
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public static double strafeMultiplier = 1;

    public final double voltageConstant = 12.3;


    String telemetry = "";


    public MecanumDrive(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        backRight = hardwareMap.get(DcMotorEx.class, "rb");

        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower){

        driveCommon(magnitude, theta, driveTurn);

        //scales
        frontLeftPower /= magnitude + Math.abs(driveTurn);
        frontRightPower /= magnitude + Math.abs(driveTurn);
        backLeftPower /= magnitude + Math.abs(driveTurn);
        backRightPower /= magnitude + Math.abs(driveTurn);


        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);


        telemetry = "" + frontLeftPower + " \n" + frontRightPower + " \n" + backLeftPower + " \n" + backRightPower;

        // 1 -1 1 -1
        // -1 -1 1 1

        // 1 -1 -1 1

    }



    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower){

        driveCommon(magnitude, theta, driveTurn);

        //scales if -1> powers >1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + Math.abs(driveTurn);
            frontRightPower /= magnitude + Math.abs(driveTurn);
            backLeftPower /= magnitude + Math.abs(driveTurn);
            backRightPower /= magnitude + Math.abs(driveTurn);
        }


        frontLeft.setPower(movementPower*frontLeftPower);
        frontRight.setPower(movementPower*frontRightPower);
        backLeft.setPower(movementPower*backLeftPower);
        backRight.setPower(movementPower*backRightPower);

        telemetry = "" + frontLeftPower + " \n" + frontRightPower + " \n" + backLeftPower + " \n" + backRightPower;
    }

    @Override
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower, double voltage){

        driveCommon(magnitude, theta, driveTurn);

        //scales
        frontLeftPower /= magnitude + Math.abs(driveTurn);
        frontRightPower /= magnitude + Math.abs(driveTurn);
        backLeftPower /= magnitude + Math.abs(driveTurn);
        backRightPower /= magnitude + Math.abs(driveTurn);


        frontLeftPower *= movementPower;
        frontRightPower *= movementPower;
        backLeftPower *= movementPower;
        backRightPower *= movementPower;

        scaleByVoltage(voltage);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


        telemetry = "" + frontLeftPower + " \n" + frontRightPower + " \n" + backLeftPower + " \n" + backRightPower;

    }


    @Override
    public void drive(double magnitude, double theta, double driveTurn, double movementPower, double voltage){

        driveCommon(magnitude, theta, driveTurn);

        //scales if -1> powers >1
        if(magnitude + Math.abs(driveTurn)>1){
            frontLeftPower /= magnitude + Math.abs(driveTurn);
            frontRightPower /= magnitude + Math.abs(driveTurn);
            backLeftPower /= magnitude + Math.abs(driveTurn);
            backRightPower /= magnitude + Math.abs(driveTurn);
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
    public String getTelemetry(){
        return telemetry;
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

    public void driveCommon(double magnitude, double theta, double driveTurn){
        theta += 45;

        //sin and cos of robot movement
        sin = Math.sin(Math.toRadians(theta)) * strafeMultiplier;
        cos = Math.cos(Math.toRadians(theta));
        maxMovement = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftPower = (magnitude * cos / maxMovement - driveTurn);
        frontRightPower = (magnitude * sin / maxMovement + driveTurn);
        backLeftPower = (magnitude * sin / maxMovement - driveTurn);
        backRightPower = (magnitude * cos / maxMovement + driveTurn);

    }

    public double totalCurrent() {
        return frontLeft.getCurrent(CurrentUnit.AMPS) + frontRight.getCurrent(CurrentUnit.AMPS) +
                backLeft.getCurrent(CurrentUnit.AMPS) + backRight.getCurrent(CurrentUnit.AMPS);
    }
}