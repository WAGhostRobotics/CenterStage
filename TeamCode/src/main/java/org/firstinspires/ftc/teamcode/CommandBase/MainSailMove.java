package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.core.Gnocchi;
import org.firstinspires.ftc.teamcode.library.commandSystem.ParallelCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.RunCommand;
import org.firstinspires.ftc.teamcode.library.commandSystem.SequentialCommand;

public class MainSailMove extends SequentialCommand {

    private double armPos;
    private double holderPos;

    public MainSailMove(double armPos, double holderPos) {
        super(new ParallelCommand(
                new RunCommand(() -> Gnocchi.mainSail.moveArm(armPos)),
                new RunCommand(() -> Gnocchi.mainSail.movePixelHolder(holderPos))),
                new ParallelCommand(
                        new RunCommand(() -> Gnocchi.mainSail.out()),
                        new Wait(200)),
                new RunCommand(() -> Gnocchi.mainSail.stop()
                )
        );
    }

}
