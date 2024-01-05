package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class PixelHolderFunc extends Command {

    boolean twoPixels;
    boolean intake;

    ElapsedTime timer;
    private final int pixelTime = 400;

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
    }

    @Override
    public void update() {
        if (intake) {
            Gnocchi.mainSail.in();
        } else {
            Gnocchi.mainSail.out();
        }
    }

    @Override
    public boolean isFinished() {
        if (twoPixels) {
            return timer.milliseconds() >= pixelTime * 2;
        }
        return timer.milliseconds() >= pixelTime;
    }
}
