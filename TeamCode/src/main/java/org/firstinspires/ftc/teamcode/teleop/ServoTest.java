package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test")
public class ServoTest extends LinearOpMode {
    Servo s1;
    double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                position += 0.0001;
            }
            else if (gamepad1.y) {
                position  -= 0.0001;
            } else if (gamepad1.b) {
                position = 0.29069999999998397;
            }

            telemetry.addData("Servo position ", position );
            telemetry.update();
            s1.setPosition(position);

        }

    }


}

