package org.firstinspires.ftc.teamcode.tuning.sensor;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.SensorDistanceEx;
import com.seattlesolvers.solverslib.hardware.SensorRevTOFDistance;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "Distance Sensor", group = "Sensor")
public class DistanceSensorTest extends LinearOpMode {
    public static double target = 10.0;
    public static double threshold = 2.0;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        SensorDistanceEx.DistanceTarget distanceTarget = new SensorDistanceEx.DistanceTarget(DistanceUnit.CM, target, threshold);
        SensorRevTOFDistance sensorDistance = new SensorRevTOFDistance(hardwareMap, "distance");

        sensorDistance.addTarget(distanceTarget);

        waitForStart();

        while(opModeIsActive()) {
            distanceTarget.setTarget(target);
            distanceTarget.setThreshold(threshold);

            telemetryData.addData("Distance (CM)", sensorDistance.getDistance(DistanceUnit.CM));
            telemetryData.addData("Target", target);
            telemetryData.addData("At Target", sensorDistance.targetReached(distanceTarget));
            telemetryData.addData("Threshold", threshold);

            telemetryData.update();
        }
    }
}
