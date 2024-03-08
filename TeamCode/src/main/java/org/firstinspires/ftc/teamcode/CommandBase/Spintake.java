package org.firstinspires.ftc.teamcode.CommandBase;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.Intake;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class Spintake extends Command {

    boolean intake;
    boolean cri;

    ElapsedTime timer;
    private int pixelTime = 1000;

    public Spintake(boolean intake, boolean cri) {
        this.intake = intake;
        this.cri = cri;
        if (intake) {
            pixelTime = 2000;
        }
    }

    @Override
    public void init() {
        if (timer == null) {
            timer = new ElapsedTime();
            timer.reset();
        }
        if (intake) {
            Gnocchi.intake.in();
        } else if (cri) {
            Gnocchi.intake.setHeight(Intake.IntakeHeight.INTAKE);
            Gnocchi.intake.out();
        } else {
            Gnocchi.intake.setHeight(Intake.IntakeHeight.AUTO_INTAKE);
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
