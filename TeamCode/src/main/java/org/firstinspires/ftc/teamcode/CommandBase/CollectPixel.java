package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;

public class CollectPixel extends SequentialCommand {

    public CollectPixel() {
        super(
                new ParallelCommand(
                        new SlidesMove(Slides.TurnValue.INTAKE.getTicks()),
                        new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.INTAKE.getPosition())),
                        new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.INTAKE.getPosition()))
                ),
                new ParallelCommand(
                        new PixelHolderFunc(true, false),
                        new RunCommand(() -> Gnocchi.intake.in()),
                        new RunCommand(() -> Gnocchi.mainSail.in())
                )
        );
    }
}
