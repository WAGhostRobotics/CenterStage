package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueOutAuto extends AutoParent{

    @Override
    public void runOpMode() throws InterruptedException {
        super.startPos = StartPos.OUT;
        super.red = false;
        super.left = false;
        super.runOpMode();
    }
}
