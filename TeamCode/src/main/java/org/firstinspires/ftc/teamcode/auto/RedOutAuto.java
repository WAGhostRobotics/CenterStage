package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedOutAuto extends AutoParent{

    @Override
    public void runOpMode() throws InterruptedException {
        super.red = true;
        super.left = true;
        super.startPos = StartPos.OUT;
        super.runOpMode();
    }
}
