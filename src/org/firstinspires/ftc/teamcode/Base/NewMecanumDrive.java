package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;

public class NewMecanumDrive extends OscarCommon{

    private static DcMotor _frontLeft, _frontRight, _backLeft, _backRight;

    public static boolean isFieldOriented = false;
    public static boolean toggleTurnSensitivity = true;
    public static boolean atTarget = false;

    private static int AutoTargetPos = 0;
    private static int AutoCurrentPos = 0;
    private static int AutoStartPos = 0;
    private static final int AUTO_MOVE_TOLERANCE = 50;

    private static final double ROTATE_SPEED_MULTIPLIER = 1.2;
    private static final double ROTATE_SPEED_EXTENDED_MULTIPLIER = 0.3;
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
    public static void driveCartesian(double xSpeed, double ySpeed, double zRotation, double gyroAngle) {
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
        boolean rightStickTranslationalDrive = true;
        boolean operatorTurning = false;

        double rotateStick = (rightStickTranslationalDrive ? gamepad.left_stick_x : gamepad.right_stick_x);
        double opRotateStick = (rightStickTranslationalDrive ? gamepad.left_stick_x : gamepad.right_stick_x);
        double lastRotateStick = rightStickTranslationalDrive ? lastGamepad.left_stick_x : gamepad.right_stick_x * ROTATE_SPEED_MULTIPLIER;
        double rightStickX = rightStickTranslationalDrive ? gamepad.right_stick_x : gamepad.left_stick_x* DRIVE_SPEED_MULTIPLIER;
        double rightStickY = rightStickTranslationalDrive ? -gamepad.right_stick_y : -gamepad.left_stick_y* DRIVE_SPEED_MULTIPLIER;

        if (!operatorTurning){
            if (toggleTurnSensitivity){
                if (gamepad.a)
                    rotateStick *= 0.4;
            } else {
                rotateStick *= rampTurning(rotateStick);
            }
        } else {
            if (Math.abs(rotateStick) > 0.01){
                if (toggleTurnSensitivity){
                    if (gamepad.a)
                        rotateStick *= 0.4;
                } else {
                    rotateStick *= rampTurning(rotateStick);
                }
            }
            if (toggleTurnSensitivity){
                if (gamepad.a)
                    rotateStick *= 0.4;
            } else {
                rotateStick *= rampTurning(rotateStick);
            }

        }


        if (gamepad.y && !lastGamepad.y) {
            isFieldOriented = !isFieldOriented;
            Gyro.reset();
        }

        if(!isFieldOriented) {
            rotateStick = Gyro.compensate(rotateStick, lastRotateStick);
        }

        if (gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right) {
            if (gamepad.dpad_down) { // backwards
                backward(gamepad.left_bumper ? .2 : 0.4);
            } else if (gamepad.dpad_left) { // left
                left(gamepad.left_bumper ? .3 : 0.5);
            } else if (gamepad.dpad_up) { // forwards
                forward(gamepad.left_bumper ? .2 : 0.4);
            } else { // right
                right(gamepad.left_bumper ? .3 : 0.5);
            }
        } else {
            Gyro.TargetHeading = Gyro.CurrentGyroHeading;
            driveCartesian(rightStickX, rightStickY, rotateStick, fieldAngle());
        }
        _telemetry.addData("Gyro Heading: ", Gyro.CurrentGyroHeading);
        _telemetry.addData("Gyro Target: ", Gyro.TargetHeading);
        _telemetry.addData("Rotate Value: ", rotateStick);
        _telemetry.addData("FieldOriented? ", isFieldOriented);
    }

    public static void forward(double power){
        driveCartesian(0, power, Gyro.getCompensation(), 0);
    }

    public static void backward(double power){ driveCartesian(0, -power, Gyro.getCompensation(), 0); }

    public static void right(double power){
        driveCartesian(power, 0, Gyro.getCompensation(), 0);
    }

    public static void left(double power){
        driveCartesian(-power, 0, Gyro.getCompensation(), 0);
    }



    public static boolean forward(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        driveCartesian(0, power, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance, true);
    }

    public static boolean backward(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        power = -power;
        distance = -distance;

        driveCartesian(0, power, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance, false);
    }

    public static boolean right(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        distance = -distance;
        driveCartesian(power, 0, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance, false);
    }

    public static boolean left(double power, int distance, int heading){
        Gyro.TargetHeading = heading;
        power = -power;
        driveCartesian(power, 0, Gyro.getCompensation(), fieldAngle());

        return atTarget(distance, true);
    }

    public static boolean turn(double speed, double heading){
        Gyro.TargetHeading = heading;
        driveCartesian(speed, speed, Gyro.getCompensation(), 0);
        return (Math.abs(Gyro.TargetHeading) - Math.abs(Gyro.CurrentGyroHeading)) < Gyro.kEpsilon;
    }

    public static boolean turn(double heading){
        return turn(0, heading);
    }

    public static boolean atTarget(int distance, boolean isForward){
        AutoStartPos = AutoStartPos == 0 ? _backLeft.getCurrentPosition() : AutoStartPos;
        AutoCurrentPos = _backLeft.getCurrentPosition();
        AutoTargetPos = AutoStartPos + distance;

        if (isForward){
            atTarget =  AutoStartPos != 0 && AutoCurrentPos > AutoTargetPos - AUTO_MOVE_TOLERANCE;
        } else {
            atTarget = AutoStartPos != 0 && AutoCurrentPos < AutoTargetPos + AUTO_MOVE_TOLERANCE;
        }

        _telemetry.addData("Start Pos: ", AutoStartPos);
        _telemetry.addData("Current Pos: ", AutoCurrentPos);
        _telemetry.addData("Target Pos: ", AutoTargetPos);
        _telemetry.addData("At Target?  ", atTarget);


        if (atTarget){
            AutoStartPos = 0;
            AutoCurrentPos = 0;
            AutoTargetPos = 0;
            return true;
        } else {
            return false;
        }
    }

    private static double rampTurning(double rotateStick){
        if ((Arm.getYPos() > -500 || Arm.getYPos() < -2300) && Arm.getXPos() < -1600)
            rotateStick = ROTATE_SPEED_EXTENDED_MULTIPLIER;
        else if ((Arm.getYPos() > -550 || Arm.getYPos() < -2350) && Arm.getXPos() < -1525)
            rotateStick = 0.3;
        else if ((Arm.getYPos() > -600 || Arm.getYPos() < -2300) && Arm.getXPos() < -1450)
            rotateStick = 0.35;
        else if ((Arm.getYPos() > -650 || Arm.getYPos() < -2250) && Arm.getXPos() < -1375)
            rotateStick = 0.4;
        else if ((Arm.getYPos() > -700 || Arm.getYPos() < -2200) && Arm.getXPos() < -1300)
            rotateStick = 0.45;
        else if ((Arm.getYPos() > -750 || Arm.getYPos() < -2150) && Arm.getXPos() < -1225)
            rotateStick = 0.5;
        else if ((Arm.getYPos() > -800 || Arm.getYPos() < -2100) && Arm.getXPos() < -1150)
            rotateStick = 0.55;
        else if ((Arm.getYPos() > -850 || Arm.getYPos() < -2050) && Arm.getXPos() < -1075)
            rotateStick = 0.6;
        else if ((Arm.getYPos() > -900 || Arm.getYPos() < -2000) && Arm.getXPos() < -1000)
            rotateStick = 0.65;
        else if ((Arm.getYPos() > -950 || Arm.getYPos() < -1950) && Arm.getXPos() < -925)
            rotateStick = 0.7;
        else if ((Arm.getYPos() > -1000 || Arm.getYPos() < -1900) && Arm.getXPos() < -850)
            rotateStick = 0.75;
        else if ((Arm.getYPos() > -1050 || Arm.getYPos() < -1850) && Arm.getXPos() < -775)
            rotateStick = 0.8;
        else if ((Arm.getYPos() > -1100 || Arm.getYPos() < -1800) && Arm.getXPos() < -700)
            rotateStick = 0.85;
        else if ((Arm.getYPos() > -1150 || Arm.getYPos() < -1750) && Arm.getXPos() < -625)
            rotateStick = 0.9;
        else if ((Arm.getYPos() > -1200 || Arm.getYPos() < -1700) && Arm.getXPos() < -550)
            rotateStick = 0.95;
        else if ((Arm.getYPos() > -1250 || Arm.getYPos() < -1650) && Arm.getXPos() < -475)
            rotateStick = 1.0;
        else if ((Arm.getYPos() > -1300 || Arm.getYPos() < -1600) && Arm.getXPos() < -400)
            rotateStick = 1.05;
        else if ((Arm.getYPos() > -1350 || Arm.getYPos() < -1550) && Arm.getXPos() < -325)
            rotateStick = 1.1;
        else if ((Arm.getYPos() > -1400 || Arm.getYPos() < -1500) && Arm.getXPos() < -250)
            rotateStick = 1.15;
        else
            rotateStick = ROTATE_SPEED_MULTIPLIER;
        return rotateStick;
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
