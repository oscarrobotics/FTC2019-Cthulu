package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

public class Arm extends OscarCommon {

    private static DcMotor _intakeCollect, _intakeArmExtend, _intakeArmVertical;
    private static Servo _dumpServo;
    private static DigitalChannel _limitSwitch;

    private static int armXTargetPos, armYTargetPos;

    private static boolean lastLimitState, hasYZeroed;

    private static BigMoveScoreState currentBmsState = BigMoveScoreState.IDLE;
    private static BigMoveGroundState currentBmgState = BigMoveGroundState.IDLE;

    private static final double DUMP_OPEN = .15;
    private static final double DUMP_CLOSE = .85;

    // extension values
    private static final int ARM_X_MAX = -3500;
    private static final int ARM_X_MIN = -800;
    private static final int ARM_X_SCORE = -2800;
    private static final int ARM_X_CRATER = -2000;
    private static final int ARM_X_INCREMENT = 30;
    private static final int ARM_X_MOVE_TOLERANCE = 25;
    private static final double ARM_X_INCREMENT_BOOOST = 5;//1 equals 100% or normal power

    // rotation values
    private static final int ARM_Y_CRATER = -200;
    private static final int ARM_Y_SCORE = -2250;
    private static final int ARM_Y_MAX = -2350;
    private static final int ARM_Y_MIN = 50;
    private static final int ARM_Y_INCREMENT = 30;
    private static final int ARM_Y_MOVE_TOLERANCE = 25;
    private static final double ARM_Y_INCREMENT_BOOOST = 3;//1 equals 100% or normal power


    private enum BigMoveScoreState {
        BEGIN,
        X_RETRACT,
        Y_RAISE,
        X_EXTEND,
        IDLE
    }

    private enum BigMoveGroundState {
        BEGIN,
        Y_CLEAR_LANDER,
        X_RETRACT,
        Y_LOWER,
        X_CRATER,
        IDLE
    }

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

        _limitSwitch = Hardware.Sensors.armLimitSwitch;
        _limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        hasYZeroed = false;
        lastLimitState = false;
    }

    public static void zeroEncoders() {
        _intakeArmVertical.setMode(STOP_AND_RESET_ENCODER);
        _intakeArmExtend.setMode(STOP_AND_RESET_ENCODER);

        _intakeArmVertical.setMode(RUN_TO_POSITION);
        _intakeArmExtend.setMode(RUN_TO_POSITION);
    }

    private static int getYPos() {
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

    private static boolean moveArmY(int setpoint, double power) {
        if (armYTargetPos != setpoint) armYTargetPos = setpoint;
        if (hasYZeroed) {
            if (setpoint < ARM_Y_MAX) { setpoint = ARM_Y_MAX; }
            if (setpoint > ARM_Y_MIN) { setpoint = ARM_Y_MIN; }
        }
        _intakeArmVertical.setTargetPosition(setpoint);
        _intakeArmVertical.setPower(power);
        int error = Math.abs(Math.abs(_intakeArmVertical.getCurrentPosition()) - Math.abs(setpoint));
        return (error < ARM_Y_MOVE_TOLERANCE);
    }

    private static boolean moveArmX(int setpoint, double power) {
        if (armXTargetPos != setpoint) armXTargetPos = setpoint;
        if (setpoint < ARM_X_MAX) { setpoint = ARM_X_MAX; }
        if (setpoint > ARM_X_MIN) { setpoint = ARM_X_MIN; }
        _intakeArmExtend.setTargetPosition(setpoint);
        _intakeArmExtend.setPower(power);
        int error = Math.abs(Math.abs(_intakeArmExtend.getCurrentPosition()) - Math.abs(setpoint));
        return (error < ARM_X_MOVE_TOLERANCE);
    }

    private static boolean YScoreHeight() { return moveArmY(ARM_Y_SCORE, 1); }

    private static boolean YMinHeight() { return moveArmY(ARM_Y_CRATER, 0.5); }

    private static boolean XExtend() { return moveArmX(ARM_X_SCORE, 1); }

    private static boolean XRetract() { return moveArmX(ARM_X_CRATER, 0.5); }

    private static void update(){
        boolean currentLimitState = !_limitSwitch.getState();
        if (lastLimitState != currentLimitState){
            _intakeArmVertical.setMode(STOP_AND_RESET_ENCODER);
            _intakeArmVertical.setMode(RUN_TO_POSITION);
            _intakeArmVertical.setTargetPosition(ARM_Y_MIN);
            hasYZeroed = true;
        }
        lastLimitState = !_limitSwitch.getState();
    }

    private static void updateTelemetry() {
        _telemetry.addLine("Arm Limit: " + (!_limitSwitch.getState() ? "Pressed, " : "Not Pressed, ") + (hasYZeroed ? "Zeroed" : "Not Zeroed"));
        _telemetry.addLine("Arm Vertical target " + armYTargetPos);
        _telemetry.addLine("Arm Vertical actual   " + _intakeArmVertical.getCurrentPosition());
        _telemetry.addLine("Arm Horizontal Target: " + armXTargetPos);
        _telemetry.addLine("Arm Horizontal Actual: " + _intakeArmExtend.getCurrentPosition());
    }

    public static void teleopControl(Gamepad gamepad, Gamepad lastGamepad) {
        double ArmXStick = gamepad.right_stick_y;
        double ArmYStick = gamepad.left_stick_y;

        boolean inStateMachine = currentBmsState != BigMoveScoreState.IDLE;

        if (!inStateMachine) {

            if (Math.abs(ArmXStick) > .1) {
                armXTargetPos += (int) (ArmXStick * (ARM_X_INCREMENT * (gamepad.right_stick_button ? ARM_X_INCREMENT_BOOOST : 1)));
            } else {
                armXTargetPos = _intakeArmExtend.getCurrentPosition();
            }
            moveArmX(armXTargetPos, 1);

            if (Math.abs(ArmYStick) > .1) {
                armYTargetPos += (int) (ArmYStick * (ARM_Y_INCREMENT * (gamepad.left_stick_button ? ARM_Y_INCREMENT_BOOOST : 1)));
            } else {
                armYTargetPos = _intakeArmVertical.getCurrentPosition();
            }
            moveArmY(armYTargetPos, 1);
        }

         if (hasYZeroed) {

             if (gamepad.x && !lastGamepad.x && currentBmgState == BigMoveGroundState.IDLE) {
                 currentBmgState = BigMoveGroundState.BEGIN;
             }

             if (gamepad.b && !lastGamepad.b && currentBmsState == BigMoveScoreState.IDLE) {
                 currentBmsState = BigMoveScoreState.BEGIN;
             }

             ground();
             score();
         }

         if (gamepad.right_trigger > 0.1){
             succ(gamepad.right_trigger);
         } else if (gamepad.left_trigger > 0.1){
             unSucc(gamepad.left_trigger);
         } else {
             _intakeCollect.setPower(0.0);
         }

        dumpMineral(gamepad.left_bumper);

         update();
         updateTelemetry();
    }

    private static void score() {
        switch (currentBmsState) {
            case BEGIN:
                newBmsState(BigMoveScoreState.X_RETRACT);
                break;
            case X_RETRACT:
                if (moveArmX(ARM_X_MIN, 1)) {
                    newBmsState(BigMoveScoreState.Y_RAISE);
                }
                break;
            case Y_RAISE:
                if (YScoreHeight()) {
                    newBmsState(BigMoveScoreState.X_EXTEND);
                }
                break;
            case X_EXTEND:
                if (XExtend()) {
                    newBmsState(BigMoveScoreState.IDLE);
                }
                break;
            case IDLE:
                // DO NOTHING
                break;
        }

    }

    private static void ground() {
        switch(currentBmgState) {
            case BEGIN:
                newBmgState(BigMoveGroundState.Y_CLEAR_LANDER);
                break;
            case Y_CLEAR_LANDER:
                if(moveArmY(getYPos() + 500, .5))
                break;
            case X_RETRACT:
                if(XRetract()) {
                    newBmgState(BigMoveGroundState.Y_LOWER);
                }
                break;
            case Y_LOWER:
                if (moveArmY(ARM_Y_CRATER, .5)) {
                    newBmgState(BigMoveGroundState.X_CRATER);
                }
                break;
            case X_CRATER:
                if(moveArmX(ARM_X_CRATER, .5)) {
                    newBmgState(BigMoveGroundState.IDLE);
            }
                break;
            case IDLE:
                break;
        }
    }

    private static void succ(double succPower) {
        _intakeCollect.setPower(Math.pow(succPower, 2));
    }

    private static void unSucc(double unSuccPower) {
        _intakeCollect.setPower(-Math.pow(unSuccPower, 2));
    }

    private static void dumpMineral(boolean state) {
        _dumpServo.setPosition(state ? DUMP_OPEN : DUMP_CLOSE);
    }

    private static void newBmsState(BigMoveScoreState newState) {
        currentBmsState = newState;
        _telemetry.addData("BMS State:", newState);
    }

    private static void newBmgState(BigMoveGroundState newState) {
        currentBmgState = newState;
    }
}
