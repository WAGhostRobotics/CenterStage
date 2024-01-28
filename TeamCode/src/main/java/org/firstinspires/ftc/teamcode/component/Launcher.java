package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    private Servo launcher;

    private final double closePos = 0.908;
    private final double openPos = 0.74;

    public void init(HardwareMap hwMap) {
        launcher = hwMap.get(Servo.class, "launcher");
        launcher.setPosition(closePos);
    }

    public void launch() {
        launcher.setPosition(openPos);
    }

    public void close() {
        launcher.setPosition(closePos);
    }

    public double getPosition() {
        return launcher.getPosition();
    }


}
