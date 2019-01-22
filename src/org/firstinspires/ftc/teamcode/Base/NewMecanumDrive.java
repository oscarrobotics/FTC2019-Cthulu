package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class NewMecanumDrive extends OscarCommon{

    private static DcMotor _frontLeft, _frontRight, _backLeft, _backRight;

    public static boolean isFieldOriented = false;

    private static int AutoTargetPos = 0;
    private static int AutoCurrentPos = 0;
    private static int AutoStartPos = 0;
    private static final int AUTO_MOVE_TOLERANCE = 100;

    private static final double ROTATE_SPEED_MULTIPLIER = 1.2;
    private static final double DRIVE_SPEED_MULTIPLIER = .9;

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
     * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param ySpeed    The robot's speed along the T axis [-1.0..1.0]. Right is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *                  positive.
     * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use
     *                  this to implement field-oriented controls.
     */
    private static void driveCartesian(double xSpeed, double ySpeed, double zRotation, double gyroAngle) {
        // Compensate for gyro angle.

        Vector2d input = new Vector2d(xSpeed, ySpeed);
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

    public static void teleopControl(Gamepad gamepad, Gamepad lastGamepad) {
        boolean leftStickTranslationalDrive = true;

        double rotateStick = leftStickTranslationalDrive ? gamepad.left_stick_x : gamepad.right_stick_x * ROTATE_SPEED_MULTIPLIER;
        double lastRotateStick = leftStickTranslationalDrive ? lastGamepad.left_stick_x : gamepad.right_stick_x * ROTATE_SPEED_MULTIPLIER;
        double rightStickX = leftStickTranslationalDrive ? gamepad.right_stick_x : gamepad.left_stick_x* DRIVE_SPEED_MULTIPLIER;
        double rightStickY = leftStickTranslationalDrive ? -gamepad.right_stick_y : -gamepad.left_stick_y* DRIVE_SPEED_MULTIPLIER;

        if (gamepad.y && !lastGamepad.y) {
            isFieldOriented = !isFieldOriented;
            Gyro.reset();
        }

        if(!isFieldOriented) {
            rotateStick = Gyro.compensate(rotateStick, lastRotateStick);
        }

        if (gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right) {
            if (gamepad.dpad_down) { // backwards
                backward(gamepad.right_bumper ? .2 : 0.4);
            } else if (gamepad.dpad_left) { // left
                left(gamepad.right_bumper ? .3 : 0.5);
            } else if (gamepad.dpad_up) { // forwards
                forward(gamepad.right_bumper ? .2 : 0.4);
            } else { // right
                right(gamepad.right_bumper ? .3 : 0.5);
            }
        } else {
            driveCartesian(rightStickX, rightStickY, rotateStick, fieldAngle());
        }
        _telemetry.addData("Gyro Heading: ", Gyro.CurrentGyroHeading);
        _telemetry.addData("Gyro Target: ", Gyro.TargetHeading);
        _telemetry.addData("Rotate Value: ", rotateStick);
        _telemetry.addData("FieldOriented? ", isFieldOriented);
    }

    public static void forward(double power){
        driveCartesian(0, power, 0, 0);
    }

    public static void backward(double power){ driveCartesian(0, -power, 0, 0); }

    public static void right(double power){
        driveCartesian(power, 0, 0, 0);
    }

    public static void left(double power){
        driveCartesian(-power, 0, 0, 0);
    }



    public static boolean forward(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        driveCartesian(0, power, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance);
    }

    public static boolean backward(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        power = -power;
        distance = -distance;
        driveCartesian(0, power, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance);
    }

    public static boolean right(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        distance = -distance;
        driveCartesian(power, 0, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance);
    }

    public static boolean left(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        power = -power;
        driveCartesian(power, 0, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance);
    }

    public static boolean turn(double speed, double heading){
        Gyro.TargetHeading = heading;
        driveCartesian(speed, speed, Gyro.getCompensation(), 0);
        return (Math.abs(Gyro.TargetHeading) - Math.abs(Gyro.CurrentGyroHeading)) < Gyro.kEpsilon;
    }

    public static boolean turn(double heading){
        return turn(0, heading);
    }

    public static boolean atTarget(int distance){
        AutoStartPos = AutoStartPos == 0 ? _backLeft.getCurrentPosition() : AutoStartPos;
        AutoCurrentPos = _backLeft.getCurrentPosition();
        AutoTargetPos = AutoStartPos + distance;

        _telemetry.addData("Start Pos: ", AutoStartPos);
        _telemetry.addData("Current Pos: ", AutoCurrentPos);
        _telemetry.addData("Target Pos: ", AutoTargetPos);

        if (AutoStartPos != 0 && Math.abs(AutoCurrentPos - AutoTargetPos) < AUTO_MOVE_TOLERANCE){
            AutoStartPos = 0;
            AutoCurrentPos = 0;
            AutoTargetPos = 0;
            return true;
        } else {
            return false;
        }
    }

    public static void updateTelemetry(int targetPos) {
        _telemetry.addLine()
                .addData("Actual", _backLeft.getCurrentPosition())
                .addData("Dest", targetPos);
    }

    public static void stop() {
        driveCartesian(0, 0, 0, 0);
    }

    private static double fieldAngle() {
        return isFieldOriented ? Gyro.CurrentGyroHeading : 0.0;
    }
}
