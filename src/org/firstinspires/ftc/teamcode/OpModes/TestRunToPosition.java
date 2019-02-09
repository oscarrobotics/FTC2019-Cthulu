package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.OpModes.TestRunToPosition.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TEST RunToPosition", group = "Oscar")
public class TestRunToPosition extends OscarBaseOp {
    int currentFR, currentFL, currentBR, currentBL, targetFR, targetFL, targetBR, targetBL;
    double avgCurrent, avgTarget;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private State mCurrentMainState;
    public static final int EPSILON = 5;
    private void newState(State newState){
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        reset();
        mCurrentMainState = newState;
    }


    public static enum State { // Ideally, these stay in order of how we use them
        STATE_INITIAL,
        STATE_WAIT,
        STATE_FORWARD,
        STATE_TURN,
        STATE_FORWARD2,
        STATE_TURN2,
        STATE_FORWARD3,
        STATE_TURN3,
        STATE_STOP
    }


    //code
    @Override
    public void start() {
        resetStartTime();
        newState(State.STATE_INITIAL);
    }

    @Override
    public void init() {
        super.init();

        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontRight = hardwareMap.dcMotor.get("rightFront");
        backLeft = hardwareMap.dcMotor.get("leftBack");
        backRight = hardwareMap.dcMotor.get("rightBack");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    public void loop() {
        currentBL = backLeft.getCurrentPosition();
        currentBR = backRight.getCurrentPosition();
        currentFL = frontLeft.getCurrentPosition();
        currentFR = frontRight.getCurrentPosition();

        targetBL = backLeft.getTargetPosition();
        targetBR = backRight.getTargetPosition();
        targetFL = frontLeft.getTargetPosition();
        targetFR = frontRight.getTargetPosition();

        avgCurrent = ((double) backLeft.getCurrentPosition() + backRight.getCurrentPosition() + frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 4;
        avgTarget = ((double) backLeft.getTargetPosition() + backRight.getTargetPosition() + frontLeft.getTargetPosition() + frontRight.getTargetPosition()) / 4;
        super.loop();
        RunStateMachine();
        telemetry.addLine("STATE: " + mCurrentMainState);
        telemetry.addData("Current Pos: ", avgCurrent);
        telemetry.addData("Target Pos: ", avgTarget);
        telemetry.addData("At Target?  ", atTarget());
    }

    private void RunStateMachine() {
        switch (mCurrentMainState) {
            case STATE_INITIAL:
                setPower(.1);
                newState(STATE_FORWARD);
                break;

            case STATE_FORWARD:
                if (forward(1000)){
                    newState(STATE_TURN);
                }
                break;

            case STATE_TURN:
                if (right(1000)){
                    newState(STATE_FORWARD2);
                }
                break;

            case STATE_FORWARD2:
                if (backward(1000)){
                    newState(STATE_TURN2);
                }
                break;

            case STATE_TURN2:
                if (left(1000)){
                    newState(STATE_STOP);
                }
                break;

            case STATE_STOP:
                setPower(0.0);
                break;
        }
    }

    public boolean forward(int position){
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public boolean backward(int position){
        position *= -1;
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(position);
        frontRight.setTargetPosition(position);
        frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public boolean right(int position){
        position *= 1.5;
        //front left and back right
        backLeft.setTargetPosition(-position);
        backRight.setTargetPosition(position);
        frontRight.setTargetPosition(-position);
        frontLeft.setTargetPosition(position);
        return atTarget();
    }

    public boolean left(int position){
        position *= 1.5;
        //front right and back left
        backLeft.setTargetPosition(position);
        backRight.setTargetPosition(-position);
        frontRight.setTargetPosition(position);
        frontLeft.setTargetPosition(-position);
        return atTarget();
    }

    public boolean atTarget(){
        return (!backRight.isBusy() || !frontRight.isBusy() || !backRight.isBusy() || !backLeft.isBusy()) || isInTolerance();
    }

    public boolean isInTolerance(){
        return Math.abs((Math.abs(targetBL) + Math.abs(targetFR)) - Math.abs((Math.abs(currentBL) + Math.abs(currentFR)))) < EPSILON ||
                Math.abs((Math.abs(targetBR) + Math.abs(targetFL)) - Math.abs((Math.abs(currentBR) + Math.abs(currentFL)))) < EPSILON;
    }

    public void reset(){
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

    public void setPower(double power){
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

}
