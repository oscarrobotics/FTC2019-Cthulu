package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

public class Lift extends OscarCommon {
    private static DcMotor _elevator;

    public static void init() {
        _elevator = Hardware.MechanismMotors.elevator;
        _elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        _elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void resetEncoders() {
        _elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
