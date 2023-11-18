package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class SlidesMove extends Command {

    int ticks;

    public SlidesMove(int ticks) {
        this.ticks = ticks;
    }

    @Override
    public void update() {
        Gnocchi.slides.update();
    }

    @Override
    public boolean isFinished() {
        return Gnocchi.slides.isFinished();
    }
}