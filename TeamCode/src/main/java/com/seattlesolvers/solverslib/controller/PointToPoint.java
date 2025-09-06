package com.seattlesolvers.solverslib.controller;

import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.geometry.Pose2d;

public class PointToPoint {
    private Controller controllerX;
    private Controller controllerY;
    private Controller controllerH;

    public double angularTolerance = 0;
    public double linearTolerance = 0;
    private Robot robot;
    private Pose2d target;

    public PointToPoint(Robot robot, SquIDController controllerX, SquIDController controllerY, SquIDController controllerH) {
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.controllerH = controllerH;
    }
    public PointToPoint(Robot robot, SquIDController controllerX, SquIDController controllerY) {
        this(robot, controllerX, controllerY, null);
    }
}