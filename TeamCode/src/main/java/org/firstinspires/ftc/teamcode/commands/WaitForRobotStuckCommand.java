package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

public class WaitForRobotStuckCommand extends CommandBase { //does not work and i don't know why
    private Follower follower;
    public WaitForRobotStuckCommand(Follower follower) {
        this.follower = follower;
    }
    @Override
    public boolean isFinished() {
        return follower.isRobotStuck();
    }
}
