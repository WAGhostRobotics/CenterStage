package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MainSail {

// rails gone :(
//    private CRServo dist1;
//    private CRServo dist2;
    private Servo pixelHold;
    private Servo arm;
    private CRServo pixelDrop;

//    private RevColorSensorV3 colorSensor;

    private final double BOARD_DIST = 50;
    private final double PERMISSIBLE_ERROR = 0.01;

    private double holdPos;
    private double armPos;

    public enum ArmPos {
        RETRACT(0.0855),
        INTAKE(0.0855),
        PLACE(0.619),
        LOW(0.1), //idk why this is here
        SPIKE(0.745);

        private double position;

        ArmPos(double position) {
            this.position = position;
        }

        public double getPosition() {return position;}
    }

    public enum HolderPos {
        RETRACT(0.0875),
        INTAKE(0.0875),
        SPIKE(0.1),
        PLACE(0.247);

        private double position;

        HolderPos(double position) {
            this.position = position;
        }

        public double getPosition() {return position;}
    }

    public void init(HardwareMap hwMap) {
//        dist1 = hwMap.get(CRServo.class, "dist1");
//        dist2 = hwMap.get(CRServo.class, "dist2");
        arm = hwMap.get(Servo.class, "arm");
        pixelHold = hwMap.get(Servo.class, "pixelHolder");
        pixelDrop = hwMap.get(CRServo.class, "pixelDrop");

        pixelDrop.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setPosition(ArmPos.RETRACT.getPosition());
        pixelHold.setPosition(HolderPos.RETRACT.getPosition());
//        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");

//        dist2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

//    // for CR servo rails
//    public void adjustDist() {
//        double error = colorSensor.getDistance(DistanceUnit.MM) - BOARD_DIST;
//
//        if (Math.abs(error) < PERMISSIBLE_ERROR) {
//            dist1.setPower(0);
//            dist2.setPower(0);
//        } else if (Math.abs(error) < 75) {
//            dist1.setPower((error > 0) ? 0.8 : -0.8);
//            dist2.setPower((error > 0) ? 0.8 : -0.8);
//        }
//
//    }

//    // for regular servo rails
//    public void moveWhole(double position) {
//        moveArm(position);
//
//        double error = BOARD_DIST - colorSensor.getDistance(DistanceUnit.MM);
//        double radiansToMove = Math.acos(Math.abs(error) / 2 / barLength);
//
//        if (Math.abs(error) > PERMISSIBLE_ERROR) {
//            if (error >= 0) {
//                dist1.setPosition(dist1.getPosition() + radiansToPos(radiansToMove));
//                dist1.setPosition(dist1.getPosition() + radiansToPos(radiansToMove));
//            } else {
//                dist1.setPosition(dist1.getPosition() - radiansToPos(radiansToMove));
//                dist2.setPosition(dist2.getPosition() - radiansToPos(radiansToMove));
//            }
//        }
//
//    }

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

//    public double getDistance() {
//        return colorSensor.getDistance(DistanceUnit.MM);
//    }

    public double posToRadians(double pos) {
        return 2 * Math.PI * pos;
    }

    public double radiansToPos(double radians) {
        return radians / 2 / Math.PI;
    }

    public boolean isFinished() {
        return Math.abs(pixelHold.getPosition() - holdPos) <= PERMISSIBLE_ERROR &&
                Math.abs(arm.getPosition() - armPos) <= PERMISSIBLE_ERROR;
    }

}
