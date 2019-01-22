package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.*;

public class Gyro extends OscarCommon {
    public static BNO055IMU.Parameters gyroParams;
    public static Orientation angles;
    public static Acceleration gravity;
    public static AngularVelocity velocity;

    public static double TargetHeading = 0.0;
    public static double CurrentGyroHeading = 0.0;
    public static double LastGyroHeading = 0.0;

    public static final double kP = 0.025;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kEpsilon = 1;

    private static double rotateValue;

    private static MiniPID gyroPid;

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

            gyroPid = new MiniPID(kP, kI, kD);
            gyroPid.setOutputLimits(-1.0, 1.0);

            _telemetry.addLine("initialized");
        }
        catch (Exception ex) {
            return false;
        }
        return true;
    }
    public static void reset() {
//        Hardware.Sensors.imu.close();
        init();
        zero();
    }


    public static void zero() {
        TargetHeading = CurrentGyroHeading;
        update();
    }

    public static double getZRotationRate() {
        return velocity.yRotationRate;
    }

    public static void setPidSetpoint(int degree) {
        TargetHeading = degree;
    }

    public static double pidCompensation() {
        return gyroPid.getOutput(CurrentGyroHeading, TargetHeading);
    }

    public static double compensate(double rotateStick, double lastRotateStick){
        if (Math.abs(rotateStick) == 0 && Math.abs(lastRotateStick) != 0) {
            Gyro.TargetHeading = (Gyro.CurrentGyroHeading + (100 * rotateStick)) % 360;
            rotateValue = Gyro.getCompensation();
        } else {
            rotateValue = rotateStick;
        }
        return rotateValue;
    }

    protected static double getCompensation(boolean isTurning) {
        double rotation = 0.0;
        double gyro = CurrentGyroHeading;
        double target = TargetHeading;
        double posError = gyro - target;
        double epsilon = 5;//3
        double minSpeed = isTurning?.4:.3;
        double maxSpeed = 0.5;//1


        if (Math.abs(posError) > 180) {
            posError = -360 * Math.signum(posError) + posError;
        }
        if (Math.abs(posError) > epsilon) {
            rotation = minSpeed + (Math.abs(posError) / 180) * (maxSpeed - minSpeed);
            rotation = rotation * Math.signum(posError);
        }

        _telemetry.addData("Gyro Output", rotation);

        return rotation;
    }

    protected static double getCompensation() {
        return getCompensation(false);
    }

    public static void update() {
        angles = Hardware.Sensors.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        gravity = Hardware.Sensors.imu.getGravity();
        velocity = Hardware.Sensors.imu.getAngularVelocity();
        CurrentGyroHeading = angles.thirdAngle;
    }
}
