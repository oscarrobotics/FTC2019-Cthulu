package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Base.Move;

import static org.firstinspires.ftc.teamcode.OpModes.TestRunToPosition.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TEST RunToPosition", group = "Oscar")
public class TestRunToPosition extends OscarBaseOp {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private State mCurrentMainState;
    public static final int EPSILON = 5;
    private void newState(State newState){
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        Move.reset();
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
        super.loop();
        RunStateMachine();
        telemetry.addLine("STATE: " + mCurrentMainState);
        telemetry.addData("Current Pos: ", Move.avgCurrent);
        telemetry.addData("Target Pos: ", Move.avgTarget);
        telemetry.addData("At Target?  ", Move.atTarget());
    }

    private void RunStateMachine() {
        switch (mCurrentMainState) {
            case STATE_INITIAL:
                newState(STATE_FORWARD);
                break;

            case STATE_FORWARD:
                if (Move.forward(0.3,1000, 0)){
                    newState(STATE_TURN);
                }
                break;

            case STATE_TURN:
                if (Move.right(0.3,1000, 0)){
                    newState(STATE_FORWARD2);
                }
                break;

            case STATE_FORWARD2:
                if (Move.backward(0.3,1000, 0)){
                    newState(STATE_TURN2);
                }
                break;

            case STATE_TURN2:
                if (Move.left(0.3,1000, 0)){
                    newState(STATE_STOP);
                }
                break;

            case STATE_STOP:
                Move.setPower(0);
                break;
        }
    }
}
