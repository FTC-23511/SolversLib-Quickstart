package org.firstinspires.ftc.teamcode.globals;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;


public class Robot extends com.seattlesolvers.solverslib.command.Robot {
    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public MotorEx FRmotor;
    public MotorEx FLmotor;
    public MotorEx BLmotor;
    public MotorEx BRmotor;
    public CRServoEx FRswervo;
    public CRServoEx FLswervo;
    public CRServoEx BLswervo;
    public CRServoEx BRswervo;

    public GoBildaPinpointDriver pinpoint;
//    public IMU imu;

    public Drive drive;

    public void init(HardwareMap hwMap) {
        // Hardware
        FRmotor = new MotorEx(hwMap, "FR");
        FLmotor = new MotorEx(hwMap, "FL");
        BLmotor = new MotorEx(hwMap, "BL");
        BRmotor = new MotorEx(hwMap, "BR");

        FRswervo = new CRServoEx(hwMap, "FR", new AbsoluteAnalogEncoder(hwMap, "FR").zero(FR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);
        FLswervo = new CRServoEx(hwMap, "FL", new AbsoluteAnalogEncoder(hwMap, "FL").zero(FL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);
        BLswervo = new CRServoEx(hwMap, "BL", new AbsoluteAnalogEncoder(hwMap, "BL").zero(BL_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);
        BRswervo = new CRServoEx(hwMap, "BR", new AbsoluteAnalogEncoder(hwMap, "BR").zero(BR_ENCODER_OFFSET), CRServoEx.RunMode.RawPower);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-75.96, 152.62, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(Pose2d.convertToPose2D(END_POSE, DistanceUnit.INCH, AngleUnit.RADIANS));


        // Subsystems
        drive = new Drive();

        // Robot/CommandScheduler configurations
        setBulkReading(hwMap, LynxModule.BulkCachingMode.MANUAL);
        register(drive);
    }

    public void initHasMovement() {

    }

    // Not needed now that we have pinpoint
//    public void initializeImu(HardwareMap hardwareMap) {
//        // IMU orientation
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//    }
}