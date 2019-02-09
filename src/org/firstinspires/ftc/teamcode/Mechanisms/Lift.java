package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

public class Lift extends OscarCommon {
    private static DcMotor _elevator;
    private static final int ELEVATOR_MAX = 4350;
    private static final int ELEVATOR_CLIMB = 4050;
    private static final int ELEVATOR_MIN = 0;
    private static final int ELEVATOR_INCREMENT = 30;

    public static void init() {
        _elevator = Hardware.MechanismMotors.elevator;
        _elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        _elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void zeroEncoders() {
        _elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void setPosition(int position){
        if (!Arm.limitOverride) {
            if (position < ELEVATOR_MIN) position = ELEVATOR_MIN;
            if (position > ELEVATOR_MAX) position = ELEVATOR_MAX;
        }
        _elevator.setTargetPosition(position);
        _elevator.setPower(1.0);
    }

    public static int getTargetPos(){
        return _elevator.getTargetPosition();
    }

    public static void adjustPos(boolean direction) {
        int curPos = getCurrentPos();
        int dirVal = direction ? 1 : -1;
        int incrPos = dirVal * ELEVATOR_INCREMENT;
        int newPos = curPos + incrPos;
        setPosition(newPos);
    }

    public static int getCurrentPos() {
        return _elevator.getCurrentPosition();
    }



    public static void runToTop(){
        setPosition(ELEVATOR_CLIMB);
    }

    public static void runToBottom(){
        setPosition(ELEVATOR_MIN);
    }

    public static void teleopControl(Gamepad gamepad) {
        boolean dPadUp = gamepad.dpad_up;
        boolean dPadDown = gamepad.dpad_down;

        if (gamepad.y) { runToTop(); }
        else if(gamepad.a) { runToBottom(); }
        else if (dPadUp) { adjustPos(true); }
        else if (dPadDown) { adjustPos(false); }
    }
}
