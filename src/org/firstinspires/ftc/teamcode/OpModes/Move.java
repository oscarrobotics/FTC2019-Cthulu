package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Base.Gyro;
import org.firstinspires.ftc.teamcode.Base.NewMecanumDrive;

import static org.firstinspires.ftc.teamcode.Base.Hardware.DriveMotors.*;

public class Move {

    public static final int EPSILON = 5;
    public static int currentFR, currentFL, currentBR, currentBL, targetFR, targetFL, targetBR, targetBL;
    public static double avgTarget, avgCurrent;

    public static boolean forward(double power, int position, double heading){
        Gyro.TargetHeading = heading;
        setPower(power);
//        NewMecanumDrive.driveCartesian(0, power, Gyro.getCompensation(), 0);
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public static boolean backward(double power, int position, double heading){
        Gyro.TargetHeading = heading;
        position *= -1;
        setPower(power);
//        NewMecanumDrive.driveCartesian(0, -power, Gyro.getCompensation(), 0);
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public static boolean right(double power, int position, double heading){
        //front left and back right
        Gyro.TargetHeading = heading;
        setPower(power);
//        NewMecanumDrive.driveCartesian(power, 0, Gyro.getCompensation(), 0);
        backLeft.setTargetPosition(-position);
        backRight.setTargetPosition(position);
        frontRight.setTargetPosition(-position);
        frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public static boolean left(double power, int position, double heading){
        //front right and back left
        Gyro.TargetHeading = heading;
        setPower(power);
//        NewMecanumDrive.driveCartesian(-power, 0, Gyro.getCompensation(), 0);
        position *= -1;
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(-position);
        frontRight.setTargetPosition(position);
        frontLeft.setTargetPosition(-position);
        return atTarget();
    }

    private static boolean atTarget(){
        avgCurrent = ((double) backLeft.getCurrentPosition() + backRight.getCurrentPosition() + frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 4;
        avgTarget = ((double) backLeft.getTargetPosition() + backRight.getTargetPosition() + frontLeft.getTargetPosition() + frontRight.getTargetPosition()) / 4;
        return (!backRight.isBusy() || !frontRight.isBusy() || !backRight.isBusy() || !backLeft.isBusy()) || isInTolerance();
    }

    public static boolean isInTolerance(){
        currentBL = backLeft.getCurrentPosition();
        currentBR = backRight.getCurrentPosition();
        currentFL = frontLeft.getCurrentPosition();
        currentFR = frontRight.getCurrentPosition();

        targetBL = backLeft.getTargetPosition();
        targetBR = backRight.getTargetPosition();
        targetFL = frontLeft.getTargetPosition();
        targetFR = frontRight.getTargetPosition();

        return Math.abs((Math.abs(targetBL) + Math.abs(targetFR)) - Math.abs((Math.abs(currentBL) + Math.abs(currentFR)))) < EPSILON ||
                Math.abs((Math.abs(targetBR) + Math.abs(targetFL)) - Math.abs((Math.abs(currentBR) + Math.abs(currentFL)))) < EPSILON;
    }

    public static void reset(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Change back to run mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void setPower(double power){
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

}
