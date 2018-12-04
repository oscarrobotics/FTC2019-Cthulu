package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

public class Arm extends OscarCommon {

    private static DcMotor _intakeCollect, _intakeArmExtend, _intakeArmVertical;

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

    public static int getArmVerticalPos() {
        return _intakeArmVertical.getCurrentPosition();
    }

    public static void moveArmVertical(int setpoint, double power) {
        _intakeArmVertical.setTargetPosition(setpoint);
        _intakeArmVertical.setPower(power);
    }

    public static void moveArmHorizontal(int setpoint, double power) {
        _intakeArmExtend.setTargetPosition(setpoint);
        _intakeArmExtend.setPower(power);
    }

    public static void succ(double succPower) {
        _intakeCollect.setPower(succPower);
    }

    public static void unSucc(double unSuccPower) {
        _intakeCollect.setPower(-unSuccPower);
    }

    public static void dumpMineral() {

    }
}
