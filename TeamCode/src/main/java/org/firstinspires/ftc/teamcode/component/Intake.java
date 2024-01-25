package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotor intake;
    private Servo height;
    private CRServo sushi;
    private final double power = 1;

    public enum IntakeHeight {
        GROUND(0),
        INTAKE(0.37),
        PIXEL1(0.1),
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
        intake = hwMap.get(DcMotor.class, "intake");
        height = hwMap.get(Servo.class, "height");
        sushi = hwMap.get(CRServo.class, "sushi");
//        sushi.setDirection(DcMotorSimple.Direction.REVERSE);
//        height.setDirection(Servo.Direction.REVERSE);
    }

    public void in() {
        intake.setPower(power);
        sushi.setPower(1);
    }

    public void out() {
        intake.setPower(-power);
        sushi.setPower(-1);
    }

    public void stop() {
        intake.setPower(0);
        sushi.setPower(0);
    }
    public void setHeight(IntakeHeight goTo) {
        height.setPosition(goTo.getPosition());
    }

    public double getHeight() {
        return height.getPosition();
    }
}
