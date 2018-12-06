package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MecanumDrive extends OscarCommon {
    
    private static DcMotor _frontLeft, _frontRight, _backLeft, _backRight;

    // Mecanum Variables
    public static double Speed = 0;
    public static double Direction = 0;
    public static double Rotation = 0;
    public static int TargetDestination = 0;

    private static double lastKnownRotJoy = 0.0;
    private static final int MaxIncrement = 100;

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

    protected static boolean AutoDrive(double Speed, double direction, double rotation, int distance, boolean rotationComp) {
        return Drive(Speed, direction, rotation, distance, true);
    }

    public static void teleopDrive(Gamepad gamepad){
        boolean rotationCorrection = true;

        double rotate = Math.pow(gamepad.left_stick_x, 3);
        double strafe = gamepad.right_stick_x;
        double forward = gamepad.right_stick_y;

        if (rotationCorrection) {
            if (rotate == 0 && lastKnownRotJoy != 0.0) {
                Gyro.TargetHeading = Gyro.CurrentGyroHeading;
            }
            lastKnownRotJoy = rotate;

            if (rotate != 0) {
                Gyro.TargetHeading = (Gyro.CurrentGyroHeading + (MaxIncrement * rotate)) % 360;
            }
        } else {
            Rotation = rotate;
        }

        // DPad mDrive
        if (gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right) {
            if (gamepad.dpad_down) { // backwards
                Speed = gamepad.right_bumper ? 0.3 : 0.5;
                Direction = Math.atan2(Speed, 0) - Math.PI / 4;
            } else if (gamepad.dpad_left) { // left
                Speed = 0.75;
                Direction = Math.atan2(0, Speed) - Math.PI / 4;
            } else if (gamepad.dpad_up) { // forwards
                Speed = gamepad.right_bumper ? 0.3 : 0.5;
                Direction = Math.atan2(-Speed, 0) - Math.PI / 4;
            } else { // right
                Speed = 0.75;
                Direction = Math.atan2(0, -Speed) - Math.PI / 4;
            }
        } else {
            Speed = Math.hypot(strafe, forward);
            Direction = Math.atan2(forward, -strafe) - Math.PI / 4;
        }
        Drive(Speed, Direction, Rotation, 0, rotationCorrection);
    }

    protected static boolean Drive(double Speed, double direction, double rotation, int autoDistance, boolean rotationCorrection) {

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


        final double v1 = Speed * Math.cos(direction) + rotation;
        final double v2 = Speed * Math.sin(direction) - rotation;
        final double v3 = Speed * Math.sin(direction) + rotation;
        final double v4 = Speed * Math.cos(direction) - rotation;

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
