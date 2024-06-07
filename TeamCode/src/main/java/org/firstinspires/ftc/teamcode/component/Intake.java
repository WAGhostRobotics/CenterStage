package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {

    private DcMotorEx intake;
    private Servo height;
    private CRServo sushi;
    private final double power = 1;

    public enum IntakeHeight {
        RETRACT(0.65),
        INTAKE(0.1916),
        AUTO_INTAKE(0.22),
        PIXEL1(0.30),
        AUTO_INTAKE_NEW(0.45), // THis one does not extend as far. So that it does not get stuck because of the team element
        PIXEL2(0.2),
        PIXEL3(0.3),
        PIXEL4(0.4),
        PIXEL5(0.5);

        double position;
        IntakeHeight(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
        height = hwMap.get(Servo.class, "height");
        sushi = hwMap.get(CRServo.class, "sushi");
//        sushi.setDirection(DcMotorSimple.Direction.REVERSE);
        height.setDirection(Servo.Direction.REVERSE);
    }

    public void in() {
        intake.setPower(power);
        sushi.setPower(1);
    }

    public void out() {
        intake.setPower(-power);
        sushi.setPower(-1);
    }

    public void slowOut() {
        intake.setPower(-0.3);
        sushi.setPower(-0.4);
    }

    public void stop() {
        intake.setPower(0);
        sushi.setPower(0);
    }

    public void up() {
        height.setPosition(height.getPosition()+0.001);
    }

    public void down() {
        height.setPosition(height.getPosition()-0.001);
    }

    public void setHeight(IntakeHeight goTo) {
        height.setPosition(goTo.getPosition());
    }

    public double getMotorCurrentDraw() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }
    public double getHeight() {
        return height.getPosition();
    }

    public void setSushi(double power) {
        sushi.setPower(power);
    }
}
