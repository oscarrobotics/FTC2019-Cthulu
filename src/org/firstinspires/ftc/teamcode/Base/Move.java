package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Move {

    private static DcMotor _frontLeft, _frontRight, _backLeft, _backRight;

    public static void init() {
        _frontLeft = Hardware.DriveMotors.frontLeft;
        _frontRight = Hardware.DriveMotors.frontRight;
        _backLeft = Hardware.DriveMotors.backLeft;
        _backRight = Hardware.DriveMotors.backRight;
    }

    public static final int EPSILON = 5;
    public static int currentFR, currentFL, currentBR, currentBL, targetFR, targetFL, targetBR, targetBL;
    public static double avgTarget, avgCurrent;

    public static boolean forward(double power, int position, double heading){
        Gyro.TargetHeading = heading;
//        setPower(power);
        NewMecanumDrive.driveCartesian(0, power, Gyro.getCompensation(), 0);
        _backLeft.setTargetPosition(position);
        _backRight.setTargetPosition(position);
        _frontRight.setTargetPosition(position);
        _frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public static boolean backward(double power, int position, double heading){
        Gyro.TargetHeading = heading;
        position *= -1;
//        setPower(power);
        NewMecanumDrive.driveCartesian(0, -power, Gyro.getCompensation(), 0);
        _backLeft.setTargetPosition(position);
        _backRight.setTargetPosition(position);
        _frontRight.setTargetPosition(position);
        _frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public static boolean right(double power, int position, double heading){
        //front left and back right
        Gyro.TargetHeading = heading;
        position *= 1.5;
//        setPower(power);
        NewMecanumDrive.driveCartesian(power, 0, Gyro.getCompensation(), 0);
        _backLeft.setTargetPosition(-position);
        _backRight.setTargetPosition(position);
        _frontRight.setTargetPosition(-position);
        _frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public static boolean left(double power, int position, double heading){
        //front right and back left
        Gyro.TargetHeading = heading;
        position *= 1.5;
//        setPower(power);
        NewMecanumDrive.driveCartesian(-power, 0, Gyro.getCompensation(), 0);
        _backLeft.setTargetPosition(position);
        _backRight.setTargetPosition(-position);
        _frontRight.setTargetPosition(position);
        _frontLeft.setTargetPosition(-position);
        return atTarget();
    }

    public static boolean atTarget(){
        avgCurrent = ((double) _backLeft.getCurrentPosition() + _backRight.getCurrentPosition() + _frontLeft.getCurrentPosition() + _frontRight.getCurrentPosition()) / 4;
        avgTarget = ((double) _backLeft.getTargetPosition() + _backRight.getTargetPosition() + _frontLeft.getTargetPosition() + _frontRight.getTargetPosition()) / 4;
        return (!_backRight.isBusy() || !_frontRight.isBusy() || !_backRight.isBusy() || !_backLeft.isBusy()) || isInTolerance();
    }

    public static boolean isInTolerance(){
        currentBL = _backLeft.getCurrentPosition();
        currentBR = _backRight.getCurrentPosition();
        currentFL = _frontLeft.getCurrentPosition();
        currentFR = _frontRight.getCurrentPosition();

        targetBL = _backLeft.getTargetPosition();
        targetBR = _backRight.getTargetPosition();
        targetFL = _frontLeft.getTargetPosition();
        targetFR = _frontRight.getTargetPosition();

        return Math.abs((Math.abs(targetBL) + Math.abs(targetFR)) - Math.abs((Math.abs(currentBL) + Math.abs(currentFR)))) < EPSILON ||
                Math.abs((Math.abs(targetBR) + Math.abs(targetFL)) - Math.abs((Math.abs(currentBR) + Math.abs(currentFL)))) < EPSILON;
    }

    public static void reset(){
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Change back to run mode
        _frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void setPower(double power){
        _backLeft.setPower(power);
        _backRight.setPower(power);
        _frontLeft.setPower(power);
        _frontRight.setPower(power);
    }

}
