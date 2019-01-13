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
    private static boolean yInDeadzone, xInDeadzone;
    private static boolean yWasInDeadzone, xWasInDeadzone;


    private static BigMoveScoreState currentBmsState = BigMoveScoreState.IDLE;
    private static BigMoveGroundState currentBmgState = BigMoveGroundState.IDLE;

    private static final double DUMP_OPEN = .15;
    private static final double DUMP_CLOSE = .85;

    // extend/retract values
    private static final int ARM_X_MAX = -3400;
    private static final int ARM_X_MIN = -775;
    private static final int ARM_X_SCORE = -3000;
    private static final int ARM_X_CRATER = -2000;
    private static final int ARM_X_MOVE_TOLERANCE = 25;
    private static final double ARM_X_MULTIPLIER = 0.25;//1 equals full power
    private static final double ARM_X_MULTIPLIER_BOOOoST = 2.5;//1 equals 100% or normal multiplier power

    // raise/lower values
    private static final int ARM_Y_CRATER = -200;
    private static final int ARM_Y_SCORE = -2300;
    private static final int ARM_Y_MAX = -2350;
    private static final int ARM_Y_MIN = 150;
    private static final int ARM_Y_MOVE_TOLERANCE = 25;
    private static final double ARM_Y_UP_MULTIPLIER = 0.6;//1 equals full power
    private static final double ARM_Y_DOWN_MULTIPLIER = 0.1;//1 equals full power
    private static final double ARM_Y_UP_BOOST = 1.5;
    private static final double ARM_Y_DOWN_BOOST = 1.75;//1 equals 100% or normal multiplier power


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

    public static int getXPos() {
        return _intakeArmExtend.getCurrentPosition();
    }

    public static int getYTargetPos() {
        return _intakeArmVertical.getTargetPosition();
    }

    public static int getXTargetPos() {
        return _intakeArmExtend.getTargetPosition();
    }

    private static void powerMoveArmY(double power) {
        if (!yInDeadzone) {
            yWasInDeadzone = false;
            if (power < 0) {
                moveArmY(ARM_Y_MAX, power);
            } else {
                if (hasYZeroed) {
                    moveArmY(ARM_Y_MIN, power);
                } else {
                    moveArmY(10000, power);
                }
            }
        } else {
            if (!yWasInDeadzone) {
                moveArmY(getYPos(), 0.5);
                yWasInDeadzone = true;
            }

        }
    }

    private static void powerMoveArmX(double power) {
        SafetyOutput armSafety = armSafety(power, getXPos(), false);
        _telemetry.addLine("X Safety: " + armSafety._isSafe + ", " + (armSafety._errorDirection == 1 ? "MAX" : "MIN"));
        if (!xInDeadzone) {
            xWasInDeadzone = false;
            if (power < 0) {
                moveArmX(ARM_X_MAX, power);
            } else {
                moveArmX(ARM_X_MIN, power);
            }
        } else {
            if (!xWasInDeadzone) {
                moveArmX(getXPos(), 0.5);
                xWasInDeadzone = true;
            }

        }
    }
    /*
    if (!armSafety._isSafe) {
            _intakeArmExtend.setTargetPosition(armSafety._errorDirection == 1 ? ARM_X_MAX : ARM_X_MIN);
            return;
        }

        if (_intakeArmExtend.getMode() != RUN_WITHOUT_ENCODER) {
            _intakeArmExtend.setMode(RUN_WITHOUT_ENCODER);
        }
        _intakeArmExtend.setPower(power);
        if(Math.abs(power) < .1) {
            _intakeArmExtend.setTargetPosition(getXPos());
            _intakeArmExtend.setMode(RUN_TO_POSITION);
        }
     */

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

    private static class SafetyOutput {
        boolean _isSafe;
        int _errorDirection;
        int _errorAmount;

        SafetyOutput(boolean isSafe, int errorDirection, int errorAmount) {
            _isSafe = isSafe;
            _errorDirection = errorDirection;
            _errorAmount = errorAmount;
        }

        SafetyOutput(boolean isSafe) {
            _isSafe = isSafe;
            _errorDirection = 0;
            _errorAmount = 0;
        }
    }

    private static SafetyOutput armSafety(double power, int curPos, boolean isYArm) {
        if (isYArm) {
            boolean limitSwitch = !_limitSwitch.getState();
            power *= -1;
            if (limitSwitch && power < -.1) return new SafetyOutput(false, -1, 0);
            if (limitSwitch && power > .1) return new SafetyOutput(true);
            if (!hasYZeroed) {
                return new SafetyOutput(!limitSwitch, -1, 0);
            } else {
                if (power > .1)  {
                    return new SafetyOutput(!(curPos <= ARM_Y_MAX), 1, Math.abs(Math.abs(ARM_Y_MAX) - Math.abs(curPos)));
                } else if (power < -.1) {
                    return new SafetyOutput(!(curPos >= ARM_Y_MIN), -1, Math.abs(Math.abs(ARM_Y_MIN) - Math.abs(curPos)));
                } else return new SafetyOutput(true);
            }
        } else {
            power *= -1;
            if (power > .1)  {
                return new SafetyOutput(!(curPos <= ARM_X_MAX), 1, Math.abs(Math.abs(ARM_X_MAX) - Math.abs(curPos)));
            } else if (power < -.1) {
                return new SafetyOutput(!(curPos >= ARM_X_MIN), 1, Math.abs(Math.abs(ARM_X_MIN) - Math.abs(curPos)));
            } else return new SafetyOutput(true);
        }
    }

    private static void updateTelemetry() {
        _telemetry.addLine("Arm Limit: " + (!_limitSwitch.getState() ? "Pressed, " : "Not Pressed, ") + (hasYZeroed ? "Zeroed" : "Not Zeroed"));
        _telemetry.addLine("Arm Vertical target " + _intakeArmVertical.getTargetPosition());
        _telemetry.addLine("Arm Vertical actual   " + _intakeArmVertical.getCurrentPosition());
        _telemetry.addLine("Arm Horizontal Target: " + _intakeArmExtend.getTargetPosition());
        _telemetry.addLine("Arm Horizontal Actual: " + _intakeArmExtend.getCurrentPosition());
    }

    public static void teleopControl(Gamepad gamepad, Gamepad lastGamepad) {
        double ArmXStick = gamepad.right_stick_y * (gamepad.right_stick_button ? (ARM_X_MULTIPLIER * ARM_X_MULTIPLIER_BOOOoST) : ARM_X_MULTIPLIER);
        double ArmYStick = gamepad.left_stick_y;

        xInDeadzone = Math.abs(gamepad.right_stick_y) < .1;
        yInDeadzone = Math.abs(gamepad.left_stick_y) < .1;

        if (ArmYStick < -.1) {
            ArmYStick *= (gamepad.left_stick_button ? (ARM_Y_UP_MULTIPLIER * ARM_Y_UP_BOOST) : ARM_Y_UP_MULTIPLIER);
        } else if (ArmYStick > .1) {
            ArmYStick *= (gamepad.left_stick_button ? (ARM_Y_DOWN_MULTIPLIER * ARM_Y_DOWN_BOOST) : ARM_Y_DOWN_MULTIPLIER);
        }

        boolean inStateMachine = currentBmsState != BigMoveScoreState.IDLE;
        if (!inStateMachine) {
            powerMoveArmX(ArmXStick);
            powerMoveArmY(ArmYStick);
        }

         if (hasYZeroed) {
             if (gamepad.b && !lastGamepad.b && currentBmgState == BigMoveGroundState.IDLE) {
                 currentBmgState = BigMoveGroundState.BEGIN;
             }
             if (gamepad.x && !lastGamepad.x && currentBmsState == BigMoveScoreState.IDLE) {
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

    public static void succ(double succPower) {
        _intakeCollect.setPower(Math.pow(succPower, 2));
    }

    public static void unSucc(double unSuccPower) {
        _intakeCollect.setPower(-Math.pow(unSuccPower, 2));
    }

    private static void dumpMineral(boolean state) {
        _dumpServo.setPosition(state ? DUMP_OPEN : DUMP_CLOSE);
    }

    private static boolean YScoreHeight() { return moveArmY(ARM_Y_SCORE, 1); }

    private static boolean YMinHeight() { return moveArmY(ARM_Y_CRATER, 0.5); }

    private static boolean XExtend() { return moveArmX(ARM_X_SCORE, 1); }

    private static boolean XRetract() { return moveArmX(ARM_X_CRATER, 0.5); }

    private static void newBmsState(BigMoveScoreState newState) {
        currentBmsState = newState;
        _telemetry.addData("BMS State:", newState);
    }

    private static void newBmgState(BigMoveGroundState newState) {
        currentBmgState = newState;
    }
}
