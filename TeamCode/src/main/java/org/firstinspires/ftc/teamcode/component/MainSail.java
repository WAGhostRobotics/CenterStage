package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MainSail {

    private Servo pixelHold;
    private Servo arm;
    private CRServo pixelDrop;

    private final double PERMISSIBLE_ERROR = 0.01;

    private double holdPos;
    private double armPos;

    public enum ArmPos {
        RETRACT(0.0855),
        INTAKE(0.0855),
        PLACE(0.619),
        MID(0.165),
        SPIKE(0.745);

        private double position;

        ArmPos(double position) {
            this.position = position;
        }

        public double getPosition() {return position;}
    }

    public enum HolderPos {
        // 0.0875
        RETRACT(0.0),
        INTAKE(0.0),
        SPIKE(0.1),
        PLACE(0.137);

        private double position;

        HolderPos(double position) {
            this.position = position;
        }

        public double getPosition() {return position;}
    }

    public void init(HardwareMap hwMap) {
        arm = hwMap.get(Servo.class, "arm");
        pixelHold = hwMap.get(Servo.class, "pixelHolder");
        pixelDrop = hwMap.get(CRServo.class, "pixelDrop");

//        pixelDrop.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setPosition(ArmPos.INTAKE.getPosition());
        pixelHold.setPosition(HolderPos.RETRACT.getPosition());

    }


    public void moveArm(double position) {
        armPos = position;
        arm.setPosition(position);
    }

    public void movePixelHolder(double position) {
        holdPos = position;
        pixelHold.setPosition(position);
    }

    public void in() {
        pixelDrop.setPower(1);
    }

    public void out() {
        pixelDrop.setPower(-1);
    }

    public void stop() {
        pixelDrop.setPower(0);
    }

    public void adjustArm(boolean up) {
        if (up) {
            armPos += 0.01;
            arm.setPosition(arm.getPosition() + 0.01);
        } else {
            armPos -= 0.01;
            arm.setPosition(arm.getPosition() - 0.01);
        }
    }

    public boolean isFinished() {
        return Math.abs(pixelHold.getPosition() - holdPos) <= PERMISSIBLE_ERROR &&
                Math.abs(arm.getPosition() - armPos) <= PERMISSIBLE_ERROR;
    }

    public boolean armAtIntake() {
        return Math.abs(arm.getPosition() - ArmPos.INTAKE.position) < 0.1;
    }

}
