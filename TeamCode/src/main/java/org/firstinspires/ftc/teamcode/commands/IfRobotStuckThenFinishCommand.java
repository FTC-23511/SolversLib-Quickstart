package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

public class IfRobotStuckThenFinishCommand extends CommandBase {
    private Follower follower;
    public IfRobotStuckThenFinishCommand(Follower follower) {
        this.follower = follower;
    }
    @Override
    public boolean isFinished() {
        return follower.isRobotStuck();
    }
}
