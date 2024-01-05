package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;

public class IntakePixel extends SequentialCommand {

    public IntakePixel() {
        super(
                new ParallelCommand(
                        new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.MID.getPosition())),
                        new SlidesMove(Slides.TurnValue.INTAKE.getTicks())
                ),
                new RunCommand(() -> Gnocchi.slides.setTargetPosition(Slides.TurnValue.SUPER_RETRACTED.getTicks())),
                new Wait(150),
                new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition())),
                new SlidesMove(Slides.TurnValue.INTAKE.getTicks())
        );
    }
}
