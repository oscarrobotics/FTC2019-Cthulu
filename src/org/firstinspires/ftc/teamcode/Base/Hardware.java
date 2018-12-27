package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

public class Hardware extends OscarCommon {

    public static class DriveMotors {
        // Drive Motors
        public static DcMotor frontLeft;
        public static DcMotor frontRight;
        public static DcMotor backLeft;
        public static DcMotor backRight;
    }

    public static class MechanismMotors {
        // Mechanism Motors
        public static DcMotor elevator;
        public static DcMotor intakeCollect;
        public static DcMotor intakeArmVertical;
        public static DcMotor intakeArmExtend;
    }

    public static class Servos {
        // Servos
        public static Servo dumpServo;
    }

    public static class Sensors {
        // Sensors
        public static I2cDeviceSynch pixyCam;
        public static BNO055IMU imu;
        public static DigitalChannel armLimitSwitch;
    }


    public static void init(HardwareMap hardwareMap) {
        // Drive Motors
        DriveMotors.frontLeft = hardwareMap.dcMotor.get("leftFront");
        DriveMotors.frontRight = hardwareMap.dcMotor.get("rightFront");
        DriveMotors.backLeft = hardwareMap.dcMotor.get("leftBack");
        DriveMotors.backRight = hardwareMap.dcMotor.get("rightBack");

        // Mechanism Motors
        MechanismMotors.elevator = hardwareMap.dcMotor.get("elevator");
        MechanismMotors.intakeCollect = hardwareMap.dcMotor.get("intakeCollect");
        MechanismMotors.intakeArmExtend = hardwareMap.dcMotor.get("intakeArmExtend");
        MechanismMotors.intakeArmVertical = hardwareMap.dcMotor.get("intakeArmVertical");

        // Servos
        Servos.dumpServo = hardwareMap.servo.get("dumpServo");

        // Sensors
        Sensors.pixyCam = hardwareMap.i2cDeviceSynch.get("pixy");

        Sensors.imu = hardwareMap.get(BNO055IMU.class, "imu");

        Sensors.armLimitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        _telemetry.addData("HARDWARE", "INIT");
    }
}
