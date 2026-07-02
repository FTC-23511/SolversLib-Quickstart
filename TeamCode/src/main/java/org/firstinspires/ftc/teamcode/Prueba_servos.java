package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import utilidades.imuEx;

@Config
@TeleOp(name = "Prueba_servos")
public class Prueba_servos extends OpMode {



    ServoEx ServoTope;

    @Override
    public void init() {

        ServoTope = new ServoEx(hardwareMap, "ServoTope");
        ServoTope.set(0);
        ServoTope.setInverted(true);


    }

    @Override
    public void loop() {
       if (gamepad1.aWasPressed()) {
           ServoTope.set(0);
       } else if (gamepad1.bWasPressed()) {
           ServoTope.set(0.5);
       }
    }


}