package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDrive extends OscarCommon{

    private static DcMotor _frontLeft, _frontRight, _backLeft, _backRight;

    // Mecanum Variables
    public static double Speed = 0;
    public static double Direction = 0;
    public static double Rotation = 0;
    public static int TargetDestination = 0;

    public static void init() {
        _frontLeft = Hardware.DriveMotors.frontLeft;
        _frontRight = Hardware.DriveMotors.frontRight;
        _backLeft = Hardware.DriveMotors.backLeft;
        _backRight = Hardware.DriveMotors.backRight;

        _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        _frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        _backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        _backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void zeroEncoders() {
        // Stop and Zero the encoder
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Change back to run mode
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    protected static boolean AutoDrive(double speed, double direction, double rotation, int distance, boolean rotationComp) {
        return Drive(speed, direction, rotation, distance, true);
    }

    protected static boolean Drive(double speed, double direction, double rotation, int autoDistance, boolean rotationCorrection) {

        if (rotation == 0) {
            rotation = Gyro.getCompensation();
        }


        if (autoDistance != 0 && TargetDestination == 0) {
            TargetDestination = _backLeft.getCurrentPosition() + autoDistance;
        }

        _telemetry.addLine()
                .addData("TargetHeading", Gyro.TargetHeading)
                .addData("ActualHeading", Gyro.CurrentGyroHeading);
        _telemetry.addLine()
                .addData("rotation", rotation);

        _telemetry.addLine()
                .addData("Actual", _backLeft.getCurrentPosition())
                .addData("Dest", TargetDestination);


        final double v1 = speed * Math.cos(direction) + rotation;
        final double v2 = speed * Math.sin(direction) - rotation;
        final double v3 = speed * Math.sin(direction) + rotation;
        final double v4 = speed * Math.cos(direction) - rotation;

        _frontLeft.setPower(v1);
        _frontRight.setPower(v2);
        _backLeft.setPower(v3);
        _backRight.setPower(v4);

        if ((TargetDestination != 0 && Math.abs(_backLeft.getCurrentPosition() - TargetDestination) < 100)
                || (direction == 0) && (Math.abs(Gyro.TargetHeading - Gyro.CurrentGyroHeading) <= 3) ) {
            TargetDestination = 0;
            return true;
        } else {
            return false;
        }
    }


}
