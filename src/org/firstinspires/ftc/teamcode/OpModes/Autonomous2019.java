package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Base.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Base.Pixy;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous2019.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: AutoStates2019", group = "Oscar")
public class Autonomous2019 extends OscarBaseOp {

    private int stateCounter = 0;
    private double speed = 0.0;
    private int distance;
    private double depotAngle;
    private int strafeDistance;
    private StartPosition lander = StartPosition.Depot;

    private int LEFT_CUBE_DISTANCE = 1400;
    private int LEFT_CUBE_ANGLE = 105;
    private int LEFT_CUBE_STRAFE = 600;

    private int CENTER_CUBE_DISTANCE = 200;
    private int CENTER_CUBE_ANGLE = 90;
    private int CENTER_CUBE_STRAFE = 1400;

    private int RIGHT_CUBE_DISTANCE = 1050;
    private int RIGHT_CUBE_ANGLE = 75;
    private int RIGHT_CUBE_STRAFE = 2000;

    public enum StartPosition {
        Crater,
        Depot,
    }

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private State mCurrentState;

    private void newState(State newState){
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
        telemetry.addData("State", mCurrentState);
        stateCounter++;
    }

    public static enum State { // Ideally, these stay in order of how we use them
        STATE_INITIAL,
        STATE_DROP,
        STATE_DETACH_LANDER,
        STATE_STRAFE_RIGHT,
        STATE_CLEAR_LANDER,
        STATE_LINEUP_MINERALS,
        STATE_BACK,
        STATE_HIT_MINERALS,
        STATE_TURN_TO_DEPOT,
        STATE_DRIVE_TO_DEPOT,
        STATE_HIT_CUBE,
        STATE_LINEUP_DEPOT,
        STATE_TURN_TO_DEPOT_FROM_CRATER,
        STATE_DRIVE_TO_DEPOT_FROM_CRATER,
        STATE_DROP_MARKER,
        STATE_RETURN_TO_CRATER,
        STATE_TURN_TO_CRATER,
        STATE_STRAFE_TO_WALL,
        STATE_CLEAR_WALL,
        STATE_DRIVE_TO_CRATER,
        STATE_FLUSH_WALL,
        STATE_APPROACH_CRATER,
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

    }

    @Override
    public void init_loop() {
        super.init_loop();
        IsAuton = true;
        telemetry.addLine("Cube X Value: " + Pixy.getCubeX());
        telemetry.addLine("Cube Position: " + Pixy.getCubePosition());

        switch (lander) {
            case Crater:
                telemetry.addLine("Crater Side Auto");
                break;
            case Depot:
                telemetry.addLine("Depot Side Auto");
                break;
        }

        if (gamepad1.dpad_up)
            lander = StartPosition.Crater;
        if (gamepad1.dpad_down)
            lander = StartPosition.Depot;
    }

    public void loop() {
        super.loop();


        telemetry.addLine("STATE: " + mCurrentState);
        switch (mCurrentState) {
            case STATE_INITIAL:
                Pixy.update();
                newState(STATE_DROP);
                break;

            case STATE_DROP:
                Lift.setPosition(2 * 2100);
                if (Math.abs(Lift.getTargetPos() - Lift.getCurrentPos()) <= 20) {
                    newState(STATE_DETACH_LANDER);
                }
                break;

            case STATE_DETACH_LANDER:
                speed = .3;
                distance = 100;
                if (NewMecanumDrive.forward(speed, distance)){
                    NewMecanumDrive.stop();
                    newState(STATE_STRAFE_RIGHT);
                }
                break;

            case STATE_STRAFE_RIGHT:
                speed = .7;
                distance = 200;
                if (NewMecanumDrive.right(speed, distance)){
                    NewMecanumDrive.stop();
                    newState(STATE_CLEAR_LANDER);
                }
                break;

            case STATE_CLEAR_LANDER:
                speed = .3;
                distance = 450;
                if (NewMecanumDrive.forward(speed, distance)){
                    NewMecanumDrive.stop();
                    newState(STATE_LINEUP_MINERALS);
                }
                break;

            case STATE_LINEUP_MINERALS:
                speed = .7;
                distance = 1400;
                if (NewMecanumDrive.right(speed, distance)){
                    Lift.runToBottom();
                    NewMecanumDrive.stop();
                    newState(STATE_BACK);
                }

                break;

            case STATE_BACK:
                speed = .25;
                distance = 900;
                if (NewMecanumDrive.backward(speed, distance)){
                    NewMecanumDrive.stop();
                    newState(STATE_HIT_MINERALS);
                }
                break;

            case STATE_HIT_MINERALS:
                speed = .4;

                switch (Pixy.getCubePosition()) {
                    case LEFT_CUBE:
                        distance = LEFT_CUBE_DISTANCE;
                        depotAngle = LEFT_CUBE_ANGLE;
                        strafeDistance = LEFT_CUBE_STRAFE;
                        if (NewMecanumDrive.forward(speed, distance)){
                            NewMecanumDrive.stop();
                            if (lander == StartPosition.Depot)
                                newState(STATE_TURN_TO_DEPOT);
                            else
                                newState(STATE_HIT_CUBE);
                        }
                        break;

                    case UNKNOWN_CUBE:
                        distance = LEFT_CUBE_DISTANCE;
                        depotAngle = LEFT_CUBE_ANGLE;
                        strafeDistance = LEFT_CUBE_STRAFE;
                        if (NewMecanumDrive.forward(speed, distance)){
                            NewMecanumDrive.stop();
                            if (lander == StartPosition.Depot)
                                newState(STATE_TURN_TO_DEPOT);
                            else
                                newState(STATE_HIT_CUBE);
                        }
                        break;

                    case CENTER_CUBE:
                        distance = CENTER_CUBE_DISTANCE;
                        depotAngle = CENTER_CUBE_ANGLE;
                        strafeDistance = CENTER_CUBE_STRAFE;
                        if (NewMecanumDrive.forward(speed, distance)){
                            NewMecanumDrive.stop();
                            if (lander == StartPosition.Depot)
                                newState(STATE_TURN_TO_DEPOT);
                            else
                                newState(STATE_HIT_CUBE);
                        }
                        break;

                    case RIGHT_CUBE:
                        distance = RIGHT_CUBE_DISTANCE;
                        depotAngle = RIGHT_CUBE_ANGLE;
                        strafeDistance = RIGHT_CUBE_STRAFE;
                        if (NewMecanumDrive.backward(speed, distance)){
                            NewMecanumDrive.stop();
                            if (lander == StartPosition.Depot)
                                newState(STATE_TURN_TO_DEPOT);
                            else
                                newState(STATE_HIT_CUBE);
                        }
                }
                break;

            case STATE_TURN_TO_DEPOT:
                speed = .0;
//                targetHeading = -depotAngle;
//                if (MecanumDrive(speed, 0, rotationComp(), 0)) {
//                    MecanumDrive(0,0,0,0);
//                }
                NewMecanumDrive.stop();
                newState(STATE_DRIVE_TO_DEPOT);
                break;

            case STATE_DRIVE_TO_DEPOT:
                if (lander == StartPosition.Depot){
                    distance = 2000;
                    speed = .5;
                } else {
                    speed = .25;
                    distance = 1000;
                }

                if (NewMecanumDrive.forward(speed, distance)) {
                    NewMecanumDrive.stop();
                    if (lander == StartPosition.Depot) {
                        newState(STATE_DROP_MARKER);
                    } else {
                        newState(STATE_HIT_CUBE);
                    }
                }
                break;

            case STATE_HIT_CUBE:
                speed = .75;
                distance = 500;
                if (NewMecanumDrive.right(speed, distance)) {
                    NewMecanumDrive.stop();
                }

                if (NewMecanumDrive.left(speed, distance)){
                    NewMecanumDrive.stop();
                    newState(STATE_LINEUP_DEPOT);
                }
                break;

            case STATE_LINEUP_DEPOT:
                speed = 0.5;
                distance = 700;
                if (NewMecanumDrive.forward(speed, distance)){
                    newState(STATE_TURN_TO_DEPOT_FROM_CRATER);
                }

            case STATE_TURN_TO_DEPOT_FROM_CRATER:
                speed = .0;
//                targetHeading = -depotAngle;
//                if (MecanumDrive(speed, 0, rotationComp(), 0)) {
//                    MecanumDrive(0,0,0,0);
//                }
                NewMecanumDrive.stop();
                newState(STATE_DRIVE_TO_DEPOT_FROM_CRATER);
                break;

            case STATE_DRIVE_TO_DEPOT_FROM_CRATER:
                speed = 0.5;
                distance = 2000;
                if (NewMecanumDrive.forward(speed, distance)){
                    newState(STATE_RETURN_TO_CRATER);
                }

            case STATE_DROP_MARKER:
                if (mStateTime.milliseconds() >= 3000){
                    Arm.succ(0);
                } else {
                    Arm.unSucc(1);
                    newState(STATE_TURN_TO_CRATER);
                }
                break;

            case STATE_TURN_TO_CRATER:
                speed = .0;
//                targetHeading = -135;
//                if (MecanumDrive(speed, 0, rotationComp(), 0)){
//                    newState(STATE_STRAFE_TO_WALL);
//                    MecanumDrive(0,0,0,0);
//
//                }
                break;

            case STATE_STRAFE_TO_WALL:
                speed = .5;
                distance = 500;
                if (NewMecanumDrive.left(speed, distance)) {
                    Arm.succ(0);
                    NewMecanumDrive.stop();
                    newState(STATE_CLEAR_WALL);
                }
                break;

            case STATE_CLEAR_WALL:
                speed = .5;
                if (NewMecanumDrive.right(speed, distance)) {
                    Arm.succ(0);
                    NewMecanumDrive.stop();
                    newState(STATE_DRIVE_TO_CRATER);
                }
                break;

            case STATE_DRIVE_TO_CRATER:
                speed = .5;
                distance = 1000;
                if (NewMecanumDrive.forward(speed, distance)) {
                    NewMecanumDrive.stop();
                    newState(STATE_FLUSH_WALL);
                }
                break;

            case STATE_FLUSH_WALL:
                speed = .5;
                distance = 600;
                if (NewMecanumDrive.left(speed, distance)) {
                    Arm.succ(0.0);
                    NewMecanumDrive.stop();
                    newState(STATE_APPROACH_CRATER);
                }
                break;

            case STATE_APPROACH_CRATER:
                speed = .3;
                distance = 3000;
                if (NewMecanumDrive.forward(speed, distance)) {
                    NewMecanumDrive.stop();
                    newState(STATE_STOP);
                }
                break;

            case STATE_STOP:
                NewMecanumDrive.stop();
                break;

        }
    }
}