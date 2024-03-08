package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedOutPlus extends PlusOneAuto{

    @Override
    public void runOpMode() throws InterruptedException {
        super.red = true;
        super.left = true;
        super.runOpMode();
    }
}
