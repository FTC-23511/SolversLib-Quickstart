package org.firstinspires.ftc.teamcode.hardware.caching;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonLynxServoController;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServoController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.teamcode.subsystem.System;

/**
 * A wrapper CRServo class for axon servos that has two goals.
 * 1) Provide caching to avoid unnecessary setPower() lynxcommands.
 * 2) Allow for easy usage of the Axon servos.
 */
public class SolversAxonServo {
    private double offset = 0;
    private double lastPower;
    private AnalogInput servoEncoder = null;
    private final PhotonCRServo crservo;

    private double powerThreshold = 0.0001;

    public SolversAxonServo(@NonNull String axonServoName, double powerThreshold) {
        this.crservo = (PhotonCRServo) hardwareMap.get(CRServo .class, axonServoName);
        this.powerThreshold = powerThreshold;
        this.lastPower = crservo.getPower();
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    /**
     * Sets the servo encoder.
     * @param axonEncoderName The analog port of the servo encoder.
     */
    public void setServoEncoder(String axonEncoderName) {
        this.servoEncoder = hardwareMap.get(AnalogInput.class, axonEncoderName);
    }

    synchronized public void setPower(double power) {
        if (Math.abs(this.lastPower - power) > this.powerThreshold) {
            this.lastPower = power;
            this.crservo.setPower(power);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.crservo.setDirection(direction);
    }

    /**
     * Returns the CURRENT position of the axon servo.
     * @return The current, actual position of the servo in radians. This is ABSOLUTE.
     */
    public double getRawPosition() {
        // (read voltage / max voltage) * 2pi + offset
        return this.servoEncoder.getVoltage() / 3.3 * Math.PI*2 + this.offset;
    }

    public void setPwm(PwmControl.PwmRange pwmRange) {
        ServoControllerEx servoController = (ServoControllerEx) crservo.getController();
        servoController.setServoPwmRange(crservo.getPortNumber(), pwmRange);
    }

        /**
         * Returns the normalized position of the axon servo.
         * @return The normalized position.
         */
    public double getPosition() {
        return System.normalize(getRawPosition());
    }

    /**
     * Sets an offset to be added to the return value of getPosition()
     * @param offset The offset
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }
}