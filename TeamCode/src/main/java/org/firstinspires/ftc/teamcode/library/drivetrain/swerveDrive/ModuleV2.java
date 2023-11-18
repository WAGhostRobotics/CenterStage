package org.firstinspires.ftc.teamcode.library.drivetrain.swerveDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.AnalogEncoder;

@Config
public class ModuleV2 {

    private DcMotor motor;
    private CRServo pivot;
    private AnalogEncoder encoder;

//    private int motorMultiplier = 1;
    private boolean changedMultiplier = false;

    private double targetAngle;


    private double error = 0;
    private double power = 0;

    public static double voltage = 12.5; //12.08

    private final double PERMISSABLE_ERROR= 3;

    public static double p=0.025, i=0.0001, d=0.0005;
    public PIDController headingController = new PIDController(p, i, d);

    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder){
        headingController.setIntegrationBounds(-10000000, 10000000);

        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ModuleV2(DcMotor motor, CRServo pivot, AnalogEncoder encoder, double p, double i, double d){
        headingController.setIntegrationBounds(-200, 200);
        this.motor = motor;
        this.pivot = pivot;
        this.encoder = encoder;
        this.p = p;
        this.i = i;
        this.d = d;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPower(double power){
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (changedMultiplier) power *= -1;
//        motor.setPower(motorMultiplier * power);
        motor.setPower(power);
    }

    public void setHold() {
//        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setServoPower(double power) {
        pivot.setPower(power);
    }

    public void setTargetAngle(double angle){
        if(normalizeDegrees(angle)!=targetAngle){
            headingController.reset();
            targetAngle = normalizeDegrees(angle);
        }
//        headingController.reset();
//        targetAngle = normalizeDegrees(angle);
    }

    public AnalogEncoder getEncoder(){
        return encoder;
    }

    public double getTargetAngle(){
        return normalizeDegrees(targetAngle);
    }

    public double getModuleAngle(){
        return normalizeDegrees(Math.toDegrees(encoder.getCurrentPosition()));
    }

    public double getModuleCurrent() {
        return ((DcMotorImplEx) motor).getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getMotorPower() {
        return motor.getPower();
    }

    public int getMotorMultiplier() {
        return 1;
//        return motorMultiplier;
    }

    public double getPower(){
        return power;
    }

    public double getError(){
        return error;
    }


    public void update(){
        headingController.setPID(p, i, d);

        double target = getTargetAngle();
        double angle = getModuleAngle();

        error = normalizeDegrees(target - angle);

//        if(Math.abs(error)>90.0){
//            angle = normalizeDegrees(angle + 180.0);
//            changedMultiplier = true;
////            setTargetAngle(target);
////            motorMultiplier = -1;
//        }
//        else {
//            changedMultiplier = false;
////            motorMultiplier = 1;
//        }

        if (changedMultiplier) {
            angle = normalizeDegrees(angle + 180); // or similar
        }

        error = normalizeDegrees(target - angle);

        if(Math.abs(error)>90.0){
            angle = normalizeDegrees(angle + 180); // or similar
            changedMultiplier = !changedMultiplier;
        }


        error = normalizeDegrees(target - angle);

        power = Range.clip(headingController.calculate(0, error), -1, 1);

        if(Double.isNaN(power)) power = 0;

        if(Math.abs(error)<=PERMISSABLE_ERROR) {
            power = 0;
        }

        pivot.setPower(power);
    }
}
