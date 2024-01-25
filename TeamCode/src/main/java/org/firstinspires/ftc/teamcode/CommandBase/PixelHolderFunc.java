package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class PixelHolderFunc extends Command {

    boolean twoPixels;
    boolean intake;

    ElapsedTime timer;
    private final int pixelTime = 1000;


    public PixelHolderFunc(boolean twoPixels, boolean intake) {
        this.twoPixels = twoPixels;
        this.intake = intake;
    }

    @Override
    public void init() {
        if (timer == null) {
            timer = new ElapsedTime();
            timer.reset();
        }
        if (intake) {
            Gnocchi.mainSail.pixelDrop.setPower(1);
        } else {
            Gnocchi.mainSail.pixelDrop.setPower(-1);
        }
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        boolean finished = timer.milliseconds() >= pixelTime;
        if (finished) {
            Gnocchi.mainSail.pixelDrop.setPower(0);
        }
        return finished;
    }
}
