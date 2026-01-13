package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

public class ClimbSubsystem extends SubsystemBase {
    private Servo climb1Right;
    private Servo climb2Left;

    public ClimbSubsystem(final HardwareMap hMap) {
        climb1Right = hMap.get(Servo.class, "climb1");
        climb2Left = hMap.get(Servo.class, "climb2");
    }

    public void climbUp() {
        climb1Right.setPosition(1.0);
        climb2Left.setPosition(0.11);
    }

    public void climbDown() {
        climb1Right.setPosition(0.52);
        climb2Left.setPosition(0.5);
    }


    //Climb 2 is on left, climb1 is on right.
    //up() and down() methods -- down to move the wheel things down and climb, up to retract
    /*
            if (gamepad1.a) { //down
            climb1pos = 0.6;
            climb2pos = 0.42;
        }
        if (gamepad1.b) { //up
            climb1pos = 1.0;
            climb2pos = 0.11;
        }
     */



}
