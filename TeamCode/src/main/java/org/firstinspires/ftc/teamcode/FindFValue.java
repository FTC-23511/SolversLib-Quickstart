package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Find F Value")
public class FindFValue extends LinearOpMode {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotorEx.class, "shooter2");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooter");

        // Cambia la dirección si uno de los motores está montado al revés
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);


        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Presiona PLAY para medir la velocidad máxima");
        telemetry.update();

        waitForStart();

        motor1.setPower(1.0);
        motor2.setPower(1.0);

        // Esperar a que alcancen velocidad estable
        sleep(4000);

        double velocity1 = Math.abs(motor1.getVelocity());
        double velocity2 = Math.abs(motor2.getVelocity());

        double averageVelocity = (velocity1 + velocity2) / 2.0;

        double fValue = 32767.0 / averageVelocity;

        motor1.setPower(0);
        motor2.setPower(0);

        while (opModeIsActive()) {
            telemetry.addData("Motor 1 (ticks/s)", velocity1);
            telemetry.addData("Motor 2 (ticks/s)", velocity2);
            telemetry.addData("Promedio (ticks/s)", averageVelocity);
            telemetry.addData("F Calculado", fValue);
            telemetry.update();
        }
    }
}