package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;

public class Climb extends SequentialCommand {

    public Climb() {
        super(
                new ParallelCommand(
                        new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.RETRACT.getPosition())),
                        new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.RETRACT.getPosition())),
                        new SlidesMove(Slides.TurnValue.CLIMB.getTicks())
                ),
                new Wait(300),
                new RunCommand(() -> Gnocchi.slides.moveToPosition(Slides.TurnValue.RETRACTED.getTicks()))
        );
    }
}
