package org.firstinspires.ftc.teamcode.CommandBase;

import org.firstinspires.ftc.teamcode.library.autoDrive.MotionPlanner;
import org.firstinspires.ftc.teamcode.library.autoDrive.math.Bezier;
import org.firstinspires.ftc.teamcode.library.commandSystem.Command;

public class FollowTrajectory extends Command {
    MotionPlanner mp;
    Bezier traj;

    public FollowTrajectory(MotionPlanner mp, Bezier traj) {
        this.mp = mp;
        this.traj = traj;
    }

    @Override
    public void init() {
        mp.startTrajectory(traj);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mp.isFinished();
    }
}
