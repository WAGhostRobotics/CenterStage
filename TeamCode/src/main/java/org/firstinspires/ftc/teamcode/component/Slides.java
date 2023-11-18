package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Slides {

    private DcMotorEx slide1;
    private DcMotorEx slide2;
    private final double POWER = 1;
    private final double ERROR = 67.5;

    private final double minPower = 0.3;
    private final double maxPower = 1;

    private double stallCurrent = 5.9;


    public enum TurnValue {
        SUPER_RETRACTED(-49),
        RETRACTED(0),
        INTAKE(540),
        PLACE(890), // 880
        CLIMB(890); //880

        int ticks;

        TurnValue(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    public void init(HardwareMap hwMap, boolean teleop) {
        if(teleop){
            slide1 = hwMap.get(DcMotorEx.class, "slide1");
            slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slide2 = hwMap.get(DcMotorEx.class, "slide2");
            slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            slide1 = hwMap.get(DcMotorEx.class, "slide1");
            slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slide2 = hwMap.get(DcMotorEx.class, "slide2");
            slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        stallCurrent = hwMap.voltageSensor.iterator().next().getVoltage()/2.2;

        setTargetPosition(0);
    }

    public void setTargetPosition(int targetPos){
        slide1.setTargetPosition(targetPos);
    }

    public int getTargetPosition(){
        return slide1.getTargetPosition();
    }

    public void update(){
        if(slide1.getTargetPosition()==Slides.TurnValue.SUPER_RETRACTED.getTicks()&& slide1.getCurrentPosition()<=0){
            slide1.setTargetPosition(0);
        }else{
            int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

            if(slide1.getCurrentPosition()> slide1.getTargetPosition()){
                multiplier = -1;
            }
            //sets power and mode
            slide1.setPower(multiplier * POWER);
            slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    public void moveToPosition(int ticks){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(slide1.getCurrentPosition()>ticks){
            multiplier = -1;
        }
        slide1.setTargetPosition(ticks);
        slide2.setTargetPosition(ticks);

        //sets power and mode
        slide1.setPower(multiplier * POWER);
        slide2.setPower(multiplier * POWER);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // move claw up by small increments
    public void moveUp(){
        slide1.setTargetPosition(slide1.getCurrentPosition() + 40 );
        slide1.setPower(0.6);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide2.setTargetPosition(slide2.getCurrentPosition() + 40 );
        slide2.setPower(0.6);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getCurrent1(){
        return slide1.getCurrent(CurrentUnit.AMPS);
    }

    public double getCurrent2(){
        return slide2.getCurrent(CurrentUnit.AMPS);
    }

    public void moveToPosition(int ticks, double power){
        int multiplier = 1;//positive if the claw needs to go up, negative if it needs to go down

        if(slide1.getCurrentPosition()>ticks){
            multiplier = -1;
        }
        slide1.setTargetPosition(ticks);
        slide2.setTargetPosition(ticks);


        //sets power and mode
        slide1.setPower(multiplier * power);
        slide2.setPower(multiplier * power);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getAdjustedPower(){
        double power = (((maxPower-minPower)*Math.abs(slide1.getTargetPosition()- slide1.getCurrentPosition()))/(100)) + minPower;

        return power;
    }


    // move claw down by small increments
    public void moveDown(){
        slide1.setTargetPosition(slide1.getCurrentPosition() - 40);
        slide1.setPower(-0.6);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide2.setTargetPosition(slide2.getCurrentPosition() - 40);
        slide2.setPower(-0.6);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getTicks(){
        return slide1.getCurrentPosition();
    }

    public boolean isBusy(){
        return slide1.isBusy();
    }

    public boolean isFinished(){
        return Math.abs(slide1.getCurrentPosition()- slide1.getTargetPosition())<=ERROR;
    }

    public boolean isStalling(){
        return slide1.getCurrent(CurrentUnit.AMPS)>stallCurrent;
    }

    public void stopArm(){
        slide1.setPower(0);
        slide2.setPower(0);
    }
}