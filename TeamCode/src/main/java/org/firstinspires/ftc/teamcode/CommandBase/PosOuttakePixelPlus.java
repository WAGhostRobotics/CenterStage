package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.component.MainSail;
import org.firstinspires.ftc.teamcode.component.Slides;
import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;

public class PosOuttakePixelPlus extends SequentialCommand {

    public PosOuttakePixelPlus() {
        super(
//                new ParallelCommand(
//                        new Spintake(true, false),
//                        new PixelHolderFunc(false, true)
//                ),
                new SlidesMove(Slides.TurnValue.PLACE_PLUS.getTicks()),
                new ParallelCommand(
                        new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(MainSail.HolderPos.PLACE.getPosition())),
                        new RunCommand(() -> Gnocchi.mainSail.moveArm(MainSail.ArmPos.PLACE.getPosition()))
                )
        );
    }
}
