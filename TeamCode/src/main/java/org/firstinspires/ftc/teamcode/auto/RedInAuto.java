package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedInAuto extends AutoParent{

    @Override
    public void runOpMode() throws InterruptedException {
        super.red = true;
        super.left = false;
        super.startPos = StartPos.IN;
        super.runOpMode();
    }
}
