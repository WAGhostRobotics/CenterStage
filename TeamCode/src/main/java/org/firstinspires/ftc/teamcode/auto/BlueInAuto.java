package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueInAuto extends AutoParent{

    @Override
    public void runOpMode() throws InterruptedException {
        super.startPos = StartPos.IN;
        super.red = false;
        super.runOpMode();
    }
}
