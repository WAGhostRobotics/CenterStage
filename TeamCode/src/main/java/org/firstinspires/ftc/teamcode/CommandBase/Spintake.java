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
    private int outTime = 500;

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
            Gnocchi.intake.setHeight(Intake.IntakeHeight.PIXEL1);
            Gnocchi.intake.in();
        } else if (cri) {
            // Get rid of extras in pluses from stack
            Gnocchi.intake.setHeight(Intake.IntakeHeight.RETRACT);
            Gnocchi.intake.out();
        } else {
            // This is to spit out the purple pixel
            Gnocchi.intake.setHeight(Intake.IntakeHeight.AUTO_INTAKE_NEW);
            Gnocchi.intake.slowOut();
        }
    }

    @Override
    public void update() {
//        if (Gnocchi.intake.getMotorCurrentDraw()>=2.20) {
//            ElapsedTime outTimer;
//            outTimer = new ElapsedTime();
//            while (outTimer.milliseconds() <= outTime) {
//                Gnocchi.intake.out();
//            }
//            timer.reset();
//            outTimer.reset();
//            Gnocchi.intake.in();
//        }

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
