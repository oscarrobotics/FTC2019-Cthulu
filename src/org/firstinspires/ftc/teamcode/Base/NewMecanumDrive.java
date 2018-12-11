package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class NewMecanumDrive {

    private static DcMotor _frontLeft, _frontRight, _backLeft, _backRight;

    private static double gyroMaxRotationRate = 0.0;
    private static double gyroAssistKp = .1;
    private static boolean gyroAssistEnabled = true;

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

    public void enableGyroAssist(double gyroMaxRotationRate, double gyroAssistKp) {
        this.gyroMaxRotationRate = gyroMaxRotationRate;
        this.gyroAssistKp = gyroAssistKp;
        this.gyroAssistEnabled = true;
    }

    public void disableGyroAssist() {
        this.gyroMaxRotationRate = 0.0;
        this.gyroAssistKp = 1.0;
        this.gyroAssistEnabled = false;
    }

    public static boolean isGyroAssistEnabled() {
        return gyroAssistEnabled;
    }

    public static double getGyroAssistPower(double rotation) {
        double error = rotation - Gyro.getZRotationRate()/gyroMaxRotationRate;
        return gyroAssistEnabled? Util.clipRange(gyroAssistKp*error): 0.0;
    }

    protected static void holonomicDrive(double x, double y, double rotation, boolean inverted, double gyroAngle) {
        x = Util.clipRange(x);
        y = Util.clipRange(y);
        rotation = Util.clipRange(rotation);

        if (inverted) {
            x = -x;
            y =-y;
        }

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));
        double x1 = x*cosA - y*sinA;
        double y1 = x*sinA + y*cosA;

        if (isGyroAssistEnabled())
        {
            rotation += getGyroAssistPower(rotation);
        }

        double[] wheelPowers = new double[4];
        wheelPowers[0] = x1 + y1 + rotation;
        wheelPowers[1] = -x1 + y1 - rotation;
        wheelPowers[2] = -x1 + y1 + rotation;
        wheelPowers[3] = x1 + y1 - rotation;
        Util.normalizeInPlace(wheelPowers);

        _frontLeft.setPower(wheelPowers[0]);
        _frontRight.setPower(wheelPowers[1]);
        _backLeft.setPower(wheelPowers[2]);
        _backRight.setPower(wheelPowers[3]);
    }

    public static void teleopControl(Gamepad gamepad) {

        double rotateStick = Math.pow(gamepad.left_stick_x, 3);
        double rightStickX = gamepad.right_stick_x;
        double rightStickY = gamepad.right_stick_y;

        // TODO: Test this!
        holonomicDrive(rightStickX, rightStickY, rotateStick, true, Gyro.CurrentGyroHeading);
    }
}
