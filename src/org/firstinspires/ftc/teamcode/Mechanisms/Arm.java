package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.OscarCommon;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

public class Arm extends OscarCommon {

    private static DcMotor _intakeCollect, _intakeArmExtend, _intakeArmVertical;
    private static Servo _dumpServo;
    private static DigitalChannel _limitSwitch;

    private static boolean lastLimitState, hasYZeroed;
    private static boolean yInDeadzone, lastYOutDeadzone, xInDeadzone;
    private static boolean xWasInDeadzone = false;
    public static boolean inStateMachine, limitOverride = false;

    public static int holdPosition;

    private static BigMoveScoreState currentBmsState = BigMoveScoreState.IDLE;
    private static BigMoveGroundState currentBmgState = BigMoveGroundState.IDLE;

    private static ElapsedTime mBmsStateTime = new ElapsedTime();
    private static ElapsedTime mBmgStateTime = new ElapsedTime();

    private static final double DUMP_OPEN = .10;
    private static final double DUMP_CLOSE = .85;

    private static final int ARM_UP_TOLERANCE = 200;
    private static final int ARM_DOWN_TOLERANCE = 150;

    // extend/retract values
    private static final int ARM_X_MAX = -3400;
    private static final int ARM_X_MIN = 0;
    private static final int ARM_X_SCORE = -2750;
    private static final int ARM_X_CRATER = -2600;
    private static final int ARM_X_MOVE_TOLERANCE = 25;
    private static final int ARM_X_MOVE_STATE_TOLERANCE = 750;
    private static final double ARM_X_MULTIPLIER = 0.25;//1 equals full power
    private static final double ARM_X_MULTIPLIER_BOOOoST = 2.5;//1 equals 100% or normal multiplier power

    // raise/lower values
    private static final int ARM_Y_CRATER = -400;
    private static final int ARM_Y_SCORE = -2350;
    private static final int ARM_Y_MAX = -2500;
    private static final int ARM_Y_MIN = 200;
    private static final int ARM_Y_MOVE_TOLERANCE = 25;
    private static final int ARM_Y_MOVE_STATE_TOLERANCE = 750;
    private static final double ARM_Y_UP_MULTIPLIER = 0.5;//1 equals full power
    private static final double ARM_Y_DOWN_MULTIPLIER = 0.2;//1 equals full power
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

        _intakeCollect.setDirection(DcMotorSimple.Direction.REVERSE);
        _intakeArmExtend.setDirection(DcMotorSimple.Direction.FORWARD);
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

        _intakeArmExtend.setTargetPosition(0);
        _intakeArmVertical.setTargetPosition(0);
    }

    public static void zeroEncoders() {
        _intakeArmVertical.setMode(STOP_AND_RESET_ENCODER);
        _intakeArmExtend.setMode(STOP_AND_RESET_ENCODER);

        _intakeArmVertical.setMode(RUN_TO_POSITION);
        _intakeArmExtend.setMode(RUN_TO_POSITION);
    }

    public static int getYPos() {
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

    private static void powerMoveArmY(double stickVal) {//when stick is outside of deadzone
       if (yInDeadzone && lastYOutDeadzone){
           holdPosition = getYPos();
           _telemetry.addData("Holding Position", holdPosition);
       } else if (!yInDeadzone){
           if (stickVal < 0){
               moveArmY(ARM_Y_MAX, stickVal);
               holdPosition = getYPos() - ARM_UP_TOLERANCE;
               _telemetry.addData("Trying to move up", stickVal);
           } else {
               if (hasYZeroed && _limitSwitch.getState()) {
                   moveArmY(ARM_Y_MIN, stickVal);
                   holdPosition = getYPos() + (!_limitSwitch.getState() ? 0 : ARM_DOWN_TOLERANCE);
                   _telemetry.addData("Trying to move down", stickVal);
               } else {
                   moveArmY(10000, stickVal);
                   holdPosition = getYPos();
                   _telemetry.addData("Trying to move down", stickVal);
               }
           }
       } else {
           if (Math.abs(Math.abs(getYPos()) - Math.abs(holdPosition)) > 50) {
               moveArmY(holdPosition, 1);
               _telemetry.addData("Doing nothing with power", holdPosition);
           } else {
               _telemetry.addData("Doing nothing without power", holdPosition);
           }
       }
    }

    private static void powerMoveArmX(double power) {
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

    public static boolean moveArmY(int setpoint, double power) {
        if (!limitOverride) {
            if (hasYZeroed) {
                if (setpoint < ARM_Y_MAX) {
                    setpoint = ARM_Y_MAX;
                }
                if (setpoint != ARM_Y_MIN && setpoint > 0) {
                    setpoint = 0;
                }
            }
            if (!_limitSwitch.getState() && setpoint > 0) {
                setpoint = 0;
            }
        }
        _intakeArmVertical.setTargetPosition(setpoint);
        _intakeArmVertical.setPower(power);
        int error = Math.abs(Math.abs(_intakeArmVertical.getCurrentPosition()) - Math.abs(setpoint));
        return (error <  (!inStateMachine ? ARM_Y_MOVE_TOLERANCE : ARM_Y_MOVE_STATE_TOLERANCE));
    }

    public static boolean moveArmX(int setpoint, double power) {
        //if (!limitOverride){
            if (setpoint < ARM_X_MAX) { setpoint = ARM_X_MAX; }
            if (setpoint > ARM_X_MIN) { setpoint = ARM_X_MIN; }
       // }

        _intakeArmExtend.setTargetPosition(setpoint);
        _intakeArmExtend.setPower(power);
        int error = Math.abs(Math.abs(_intakeArmExtend.getCurrentPosition()) - Math.abs(setpoint));
        return (error < (!inStateMachine ? ARM_X_MOVE_TOLERANCE : ARM_X_MOVE_STATE_TOLERANCE));
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

    private static boolean armSafety(double power, int curPos) {
            boolean limitSwitch = !_limitSwitch.getState();
            power *= -1;
            curPos *= -1;
            if (limitSwitch && power < 0) return false;
            if (limitSwitch && power > 0) return true;
            if (!hasYZeroed) {
                return true;
            } else {
                if (power > 0)  {
                    return ((curPos >= ARM_Y_MAX - 500));
                } else if (power < 0) {
                    return (!(curPos <= ARM_Y_MIN));
                } else return true;
            }
    }

    private static void updateTelemetry() {
        _telemetry.addLine("Arm Limit: " + (!_limitSwitch.getState() ? "Pressed, " : "Not Pressed, ") + (hasYZeroed ? "Zeroed" : "Not Zeroed"));
        _telemetry.addLine("Arm Vertical target " + _intakeArmVertical.getTargetPosition());
        _telemetry.addLine("Arm Vertical actual   " + _intakeArmVertical.getCurrentPosition());
        _telemetry.addLine("Arm Horizontal Target: " + _intakeArmExtend.getTargetPosition());
        _telemetry.addLine("Arm Horizontal Actual: " + _intakeArmExtend.getCurrentPosition());
    }

    public static void teleopControl(Gamepad gamepad, Gamepad lastGamepad, Gamepad driver) {
        double ArmXStick = gamepad.right_stick_y * (gamepad.right_stick_button ? (ARM_X_MULTIPLIER * ARM_X_MULTIPLIER_BOOOoST) : ARM_X_MULTIPLIER);
        double ArmYStick = gamepad.left_stick_y;

        xInDeadzone = Math.abs(gamepad.right_stick_y) < .1;
        yInDeadzone = Math.abs(gamepad.left_stick_y) < .1;
        lastYOutDeadzone = Math.abs(lastGamepad.left_stick_y) > .1;

        if (ArmYStick < -.1) {
            ArmYStick *= (gamepad.left_stick_button ? (ARM_Y_UP_MULTIPLIER * ARM_Y_UP_BOOST) : ARM_Y_UP_MULTIPLIER);
        } else if (ArmYStick > .1) {
            ArmYStick *= (gamepad.left_stick_button ? (ARM_Y_DOWN_MULTIPLIER * ARM_Y_DOWN_BOOST) : ARM_Y_DOWN_MULTIPLIER);
        }

        inStateMachine = currentBmsState != BigMoveScoreState.IDLE || currentBmgState != BigMoveGroundState.IDLE;
        if (!inStateMachine) {
            powerMoveArmX(ArmXStick);
            powerMoveArmY(ArmYStick);
            /*if (driver.b)
                limitOverride = true;
            else
                limitOverride = false;
                */
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
             succ(gamepad.right_trigger * 0.9);
         } else if (gamepad.left_trigger > 0.1){
             unSucc(gamepad.left_trigger * 0.9);
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
                if (Math.abs(getYPos()) < 1800 || mBmsStateTime.milliseconds() > 1000) {
                    if (XRetract()) {
                        newBmsState(BigMoveScoreState.Y_RAISE);
                    }
                } else {
                    newBmsState(BigMoveScoreState.IDLE);
                }
                break;
            case Y_RAISE:
                if (YScoreHeight() || mBmsStateTime.milliseconds() > 1000) {
                    holdPosition = ARM_Y_SCORE;
                    newBmsState(BigMoveScoreState.X_EXTEND);
                }
                break;
            case X_EXTEND:
                if (XExtend() || mBmsStateTime.milliseconds() > 1000) {
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
                if(moveArmY(-2000, .5) || mBmgStateTime.milliseconds() > 1000){
                    newBmgState(BigMoveGroundState.X_RETRACT);
                }
                break;
            case X_RETRACT:
                if(XRetract() || mBmgStateTime.milliseconds() > 1000) {
                    newBmgState(BigMoveGroundState.Y_LOWER);
                }
                break;
            case Y_LOWER:
                if (moveArmY(ARM_Y_CRATER, .5) || mBmgStateTime.milliseconds() > 1000) {
                    holdPosition = ARM_Y_CRATER;
                    newBmgState(BigMoveGroundState.X_CRATER);
                }
                break;
            case X_CRATER:
                if(YMinHeight() || mBmgStateTime.milliseconds() > 1000) {
                    newBmgState(BigMoveGroundState.IDLE);
                }
                break;
            case IDLE:
                break;
        }
    }

    public static void succ(double succPower) { _intakeCollect.setPower(Math.pow(succPower, 3)); }

    public static void unSucc(double unSuccPower) {
        _intakeCollect.setPower(-Math.pow(unSuccPower, 3));
    }

    public static void autoIntake(double power) {
        _intakeCollect.setPower(power);
    }

    private static void dumpMineral(boolean state) {
        _dumpServo.setPosition(state ? DUMP_OPEN : DUMP_CLOSE);
    }

    private static boolean YScoreHeight() { return moveArmY(ARM_Y_SCORE, 0.75); }

    private static boolean YMinHeight() { return moveArmY(ARM_Y_CRATER, 0.5); }

    private static boolean XExtend() { return moveArmX(ARM_X_SCORE, 0.5); }

    private static boolean XRetract() { return moveArmX(-1600, 0.5); }

    public static boolean zeroY(){
        moveArmX(-1200, 0.5);
        _telemetry.addData("Limit: ", !_limitSwitch.getState());
        if (!_limitSwitch.getState()){
            update();
            return true;
        } else {
            moveArmY(2000, 0.5);
            return false;
        }
    }

    public static boolean autoScoreGold(){
        if (moveArmY(-2500,0.75) && moveArmX(-3100, 0.75)){
            return true;
        } else return false;
    }

    public static  boolean return1(){
        if (Arm.moveArmY(-2000, 0.5)) {
            if (Arm.moveArmX(-1600, 0.5)) {
                return true;
            }
        }
        return false;
    }

    public static  boolean return2(){
        if (Arm.moveArmY(-800, 0.5)) {
            if (Arm.moveArmX(0, 0.5)) {
                return true;
            }
        }
        return false;
    }

    private static void newBmsState(BigMoveScoreState newState) {
        currentBmsState = newState;
        mBmsStateTime.reset();
        _telemetry.addData("BMS State:", newState);
    }

    private static void newBmgState(BigMoveGroundState newState) {
        currentBmgState = newState;
        mBmgStateTime.reset();
        _telemetry.addData("BGS State:", newState);
    }

    public static void stop(){
        _intakeArmVertical.setPower(0);
        _intakeArmExtend.setPower(0);
    }
}
