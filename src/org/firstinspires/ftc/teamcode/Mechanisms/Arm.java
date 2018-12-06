package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

public class Arm extends OscarCommon {

    private static DcMotor _intakeCollect, _intakeArmExtend, _intakeArmVertical;
    private static Servo _dumpServo;

    private int armYtargetPos = 0;
    private int armExtendTargetPos = 0;

    private double rightTrigger;
    private double leftTrigger;
    private boolean contSpin = false;
    private boolean leftTriggerPressed = false;
    private boolean verticalArmLimitWasPressed = false;
    private boolean wasArmLimetEverPressed = false;

    private static final double DUMP_OPEN = .7;
    private static final int ARMYPOSITIONSCORE = 2750;
    private static final int ARMEXTENDSCORE = 2800;
    private static final int ARMYPOSITIONPARALLEL = 100;
    private static final int ARMEXTENDPARALLEL = 1350;
    private static final int ARMEXTENDMAX = 3000;
    private static final int ARMVERTMAX = 2850;
    private static final int ARMEXTENDMIN = 3000;
    private static final int ARMVERTMIN = 2850;

    public static void init() {
        _intakeCollect = Hardware.MechanismMotors.intakeCollect;
        _intakeArmExtend = Hardware.MechanismMotors.intakeCollect;
        _intakeArmVertical = Hardware.MechanismMotors.intakeCollect;

        _intakeCollect.setDirection(DcMotorSimple.Direction.FORWARD);
        _intakeArmExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        _intakeArmVertical.setDirection(DcMotorSimple.Direction.FORWARD);

        _intakeCollect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _intakeArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _intakeArmVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public static void moveVertical(int setpoint, double power) {
        _intakeArmVertical.setTargetPosition(setpoint);
        _intakeArmVertical.setPower(power);
    }

    public static void moveHorizontal(int setpoint, double power) {
        _intakeArmExtend.setTargetPosition(setpoint);
        _intakeArmExtend.setPower(power);
    }

    public static void maxHeight() {
        _intakeArmVertical.setTargetPosition(-ARMVERTMAX);
        _intakeArmVertical.setPower(0.75);
    }

    public static void minHeight() {
        _intakeArmVertical.setTargetPosition(-ARMVERTMIN);
        _intakeArmVertical.setPower(0.5);
    }

    public static void extend() {
        _intakeArmExtend.setTargetPosition(-ARMEXTENDMAX);
        _intakeArmExtend.setPower(0.5);
    }

    public static void retract() {
        _intakeArmExtend.setTargetPosition(-ARMEXTENDMIN);
        _intakeArmExtend.setPower(0.5);
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
        _intakeCollect.setPower(succPower);
    }

    public static void unSucc(double unSuccPower) {
        _intakeCollect.setPower(-unSuccPower);
    }

    public static void dumpMineral() {
        _dumpServo.setPosition(DUMP_OPEN);
    }
}
