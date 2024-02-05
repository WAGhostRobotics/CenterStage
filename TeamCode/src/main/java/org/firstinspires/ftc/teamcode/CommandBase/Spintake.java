package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class Spintake extends Command {

    boolean intake;

    ElapsedTime timer;
    private int pixelTime = 600;

    public Spintake(boolean intake) {
        this.intake = intake;
    }

    @Override
    public void init() {
        if (timer == null) {
            timer = new ElapsedTime();
            timer.reset();
        }
        if (intake) {
            Gnocchi.intake.in();
        } else {
            Gnocchi.intake.setHeight(Intake.IntakeHeight.INTAKE);
            Gnocchi.intake.slowOut();
        }
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        boolean finished = timer.milliseconds() >= pixelTime;
        if (finished) {
            Gnocchi.intake.stop();
            Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT);
        }
        return finished;
    }
}
