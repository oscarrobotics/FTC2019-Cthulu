package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

public class Arm extends OscarCommon {

    private static DcMotor _intakeCollect, _intakeArmExtend, _intakeArmVertical;
    private static Servo _dumpServo;

    private static int armXTargetPos, armYTargetPos;

    private static final double DUMP_OPEN = .15;
    private static final double DUMP_CLOSE = .85;

    // extension values
    private static final int ARM_X_MAX = 3000;
    private static final int ARM_X_MIN = 3000;
    private static final int ARM_X_SCORE = 2800;
    private static final int ARM_X_PARALLEL = 1350;
    private static final int ARM_X_INCREMENT = 30;

    // rotation values
    private static final int ARM_Y_POSITIONPARALLEL = 100;
    private static final int ARM_Y_POSITIONSCORE = 2750;
    private static final int ARM_Y_MAX = 2850;
    private static final int ARM_Y_MIN = 2850;
    private static final int ARM_Y_INCREMENT = 30;

    public static void init() {
        _intakeCollect = Hardware.MechanismMotors.intakeCollect;
        _intakeArmExtend = Hardware.MechanismMotors.intakeArmExtend;
        _intakeArmVertical = Hardware.MechanismMotors.intakeArmVertical;

        _intakeCollect.setDirection(DcMotorSimple.Direction.FORWARD);
        _intakeArmExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        _intakeArmVertical.setDirection(DcMotorSimple.Direction.FORWARD);

        _intakeCollect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _intakeArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _intakeArmVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _dumpServo = Hardware.Servos.dumpServo;
        _dumpServo.setPosition(DUMP_CLOSE);
    }

    public static void zeroEncoders() {
        _intakeArmVertical.setMode(STOP_AND_RESET_ENCODER);
        _intakeArmExtend.setMode(STOP_AND_RESET_ENCODER);

        _intakeArmVertical.setMode(RUN_TO_POSITION);
        _intakeArmExtend.setMode(RUN_TO_POSITION);
    }

    public static int getVerticalPos() {
        return _intakeArmVertical.getCurrentPosition();
    }

    public static int getHorizontalPos() {
        return _intakeArmExtend.getCurrentPosition();
    }

    public static int getVerticalTargetPos() {
        return _intakeArmVertical.getTargetPosition();
    }

    public static int getHorizontalTargetPos() {
        return _intakeArmExtend.getTargetPosition();
    }

    public static void moveArmY(int setpoint, double power) {
        _intakeArmVertical.setTargetPosition(setpoint);
        _intakeArmVertical.setPower(power);
    }

    public static void moveArmX(int setpoint, double power) {
        _intakeArmExtend.setTargetPosition(setpoint);
        _intakeArmExtend.setPower(power);
    }

    public static void maxHeight() {
        moveArmY(-ARM_Y_POSITIONSCORE, 0.75);
    }

    public static void minHeight() {
        moveArmY(-ARM_Y_POSITIONPARALLEL, 0.5);
    }

    public static void extend() {
        moveArmX(-ARM_X_MAX, 0.5);
    }

    public static void retract() {
        moveArmX(-ARM_X_PARALLEL, 0.5);
    }

    public static void teleopControl(Gamepad gamepad) {
        double ArmXStick = gamepad.right_stick_y;
        double ArmYStick = gamepad.left_stick_y;

        if (Math.abs(ArmXStick) > .1) { // deadzone
            armXTargetPos += (int) (ArmXStick * ARM_X_INCREMENT);
        } else {
            armXTargetPos = _intakeArmExtend.getCurrentPosition();
        }
        moveArmX(armXTargetPos, 0.5);

         if (Math.abs(ArmYStick) > .1) {
            armYTargetPos += (int) (ArmYStick * ARM_Y_INCREMENT);
         } else {
            armYTargetPos = _intakeArmVertical.getCurrentPosition();
         }
         moveArmY(armYTargetPos, 0.5);

         if (gamepad.right_trigger > 0.1){
             succ(gamepad.right_trigger);
         } else if (gamepad.left_trigger > 0.1){
             unSucc(gamepad.left_trigger);
         } else {
             _intakeCollect.setPower(0.0);
         }

        dumpMineral(gamepad.left_bumper);
    }

    public static void score() {
        maxHeight();
        extend();
    }

    public static void ground() {
        minHeight();
        retract();
    }

    public static void succ(double succPower) {
        _intakeCollect.setPower(Math.pow(succPower, 2));
    }

    public static void unSucc(double unSuccPower) {
        _intakeCollect.setPower(-Math.pow(unSuccPower, 2));
    }

    public static void dumpMineral(boolean state) {
        if (state)
            _dumpServo.setPosition(DUMP_OPEN);
        else
            _dumpServo.setPosition(DUMP_CLOSE);
    }
}
