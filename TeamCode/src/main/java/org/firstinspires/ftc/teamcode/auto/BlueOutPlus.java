package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueOutPlus extends PlusOneAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.red = false;
        super.left = false;
        super.runOpMode();
    }
}
