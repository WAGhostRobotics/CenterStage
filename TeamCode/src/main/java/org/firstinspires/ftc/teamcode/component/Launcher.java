package org.firstinspires.ftc.teamcode.component;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    private Servo launcher;

    private final double closePos = 0;
    private final double openPos = 0.2;

    public void init(HardwareMap hwMap) {
        launcher = hwMap.get(Servo.class, "launcher");
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
