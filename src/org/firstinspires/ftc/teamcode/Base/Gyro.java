package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public class Gyro extends OscarCommon {
    public static BNO055IMU.Parameters gyroParams;
    public static Orientation angles;
    public static Acceleration gravity;
    public static AngularVelocity velocity;

    public static double TargetHeading = 0.0;
    public static double CurrentGyroHeading = 0.0;

    public static boolean init(){
        try {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            gyroParams = new BNO055IMU.Parameters();
            gyroParams.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            gyroParams.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            gyroParams.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            gyroParams.loggingEnabled      = true;
            gyroParams.loggingTag          = "IMU";

            // Gyro stuff
            ////NEGATIVE = clockwise
            Hardware.Sensors.imu.initialize(gyroParams);
            _telemetry.addLine("initialized");
        }
        catch (Exception ex) {
            return false;
        }
        return true;
    }

    public static void zero() {
        update();
        TargetHeading = CurrentGyroHeading;
    }

    public static double getZRotationRate() {
        return velocity.zRotationRate;
    }

    protected static double getCompensation(boolean isTurning) {
        double rotation = 0.0;
        double gyro = CurrentGyroHeading;
        double target = TargetHeading;
        double posError = gyro - target;
        double epsilon = 3;
        double minSpeed = isTurning?.45:.2; // was 0.12
        double maxSpeed = 1;

        if (Math.abs(posError) > 180) {
            posError = -360 * Math.signum(posError) + posError;
        }
        if (Math.abs(posError) > epsilon) {
            rotation = minSpeed + (Math.abs(posError) / 180) * (maxSpeed - minSpeed);
            rotation = rotation * Math.signum(posError);
        }
        return -rotation;
    }

    protected static double getCompensation() {
        return getCompensation(false);
    }

    public static void update() {
        angles = Hardware.Sensors.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        gravity = Hardware.Sensors.imu.getGravity();
        velocity = Hardware.Sensors.imu.getAngularVelocity();
        CurrentGyroHeading = angles.firstAngle;
    }
}
