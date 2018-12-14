package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class NewMecanumDrive extends OscarCommon{

    private static DcMotor _frontLeft, _frontRight, _backLeft, _backRight;

    public static boolean isFieldOriented;

    public static void init() {
        _frontLeft = Hardware.DriveMotors.frontLeft;
        _frontRight = Hardware.DriveMotors.frontRight;
        _backLeft = Hardware.DriveMotors.backLeft;
        _backRight = Hardware.DriveMotors.backRight;

        _frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        _frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
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

    /**
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
     * from its angle or rotation rate.
     *
     * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
     * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                  positive.
     * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use
     *                  this to implement field-oriented controls.
     */
    private static void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {

    //        ySpeed = limit(ySpeed);
    //        ySpeed = applyDeadband(ySpeed, m_deadband);
    //        xSpeed = limit(xSpeed);
    //        xSpeed = applyDeadband(xSpeed, m_deadband);

        // Compensate for gyro angle.
        Vector2d input = new Vector2d(ySpeed, xSpeed);
        input.rotate(-gyroAngle);

        double[] wheelSpeeds = new double[4];
        double frontLeftPow = input.x + input.y + zRotation;
        double frontRightPow= input.x - input.y + zRotation;
        double backLeftPow = -input.x + input.y + zRotation;
        double backRightPow = -input.x - input.y + zRotation;

        Util.normalize(wheelSpeeds);

        _frontLeft.setPower(frontLeftPow);
        _frontRight.setPower(frontRightPow);
        _backLeft.setPower(backLeftPow);
        _backRight.setPower(backRightPow);
    }

    /**
     * Drive method for Mecanum platform.
     *
     * <p>Angles are measured counter-clockwise from straight ahead. The speed at which the robot
     * drives (translation) is independent from its angle or rotation rate.
     *
     * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
     * @param angle     The angle around the Z axis at which the robot drives in degrees [-180..180].
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                  positive.
     */
    @SuppressWarnings("ParameterName")
    public static void drivePolar(double magnitude, double angle, double zRotation) {
        driveCartesian(magnitude * Math.sin(angle * (Math.PI / 180.0)),
                magnitude * Math.cos(angle * (Math.PI / 180.0)), zRotation, 0.0);
    }

//    public static void teleopControl(Gamepad gamepad) {
//        double rotateStick = Math.pow(gamepad.left_stick_x, 3);
//        double rightStickX = gamepad.right_stick_x;
//        double rightStickY = -gamepad.right_stick_y;
//
//        driveCartesian(rightStickX, rightStickY, rotateStick, Gyro.CurrentGyroHeading);
//    }

    public static void teleopControl(Gamepad gamepad, Gamepad lastGamepad) {
        double rotateStick = Math.pow(gamepad.left_stick_x, 3);
        double rightStickX = gamepad.right_stick_x;
        double rightStickY = -gamepad.right_stick_y;

        if (gamepad.y != lastGamepad.y) {
            Gyro.zero();
            isFieldOriented = !isFieldOriented;
        }
        _telemetry.addData("FieldOriented", isFieldOriented);
        driveCartesian(rightStickX, rightStickY, rotateStick, isFieldOriented ? Gyro.CurrentGyroHeading : 0.0);
    }
}
