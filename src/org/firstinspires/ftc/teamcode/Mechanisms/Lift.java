package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

public class Lift extends OscarCommon {
    private static DcMotor _elevator;
    private static final int ELEVATOR_MAX = 4000;
    private static final int ELEVATOR_MIN = 0;

    public static void init() {
        _elevator = Hardware.MechanismMotors.elevator;
        _elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        _elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void resetEncoders() {
        _elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void setPosition(int position){
        _elevator.setTargetPosition(position);
    }

    public static int getTargetPos(){
        return _elevator.getTargetPosition();
    }

    public static int getCurrentPos(){
        return _elevator.getCurrentPosition();
    }

    public static void runToTop(){
        setPosition(ELEVATOR_MAX);
    }

    public static void runToBottom(){
        setPosition(ELEVATOR_MIN);
    }


}
