package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Base.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Base.Pixy;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous2019.CraterState.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous2019.DepotState.*;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous2019.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: AutoStates2019", group = "Oscar")
public class Autonomous2019 extends OscarBaseOp {

    private double speed = 0.0;
    public int distance;
    public int heading;
    public double depotAngle;
    public int strafeDistance;
    private StartPosition lander = StartPosition.Crater;
    public Pixy.CubePosition cubePosition = Pixy.CubePosition.LEFT_CUBE;

    private int LEFT_CUBE_DISTANCE = 1500;
    private int LEFT_CUBE_CRATER_DISTANCE = 1200;
    private int LEFT_CUBE_ANGLE = 105;
    private int LEFT_CUBE_STRAFE = 600;

    private int CENTER_CUBE_DISTANCE = 300;
    private int CENTER_CUBE_CRATER_DISTANCE = 200;
    private int CENTER_CUBE_ANGLE = 85;
    private int CENTER_CUBE_STRAFE = 1100;

    private int RIGHT_CUBE_DISTANCE = 1100;
    private int RIGHT_CUBE_CRATER_DISTANCE = 800;
    private int RIGHT_CUBE_ANGLE = 65;
    private int RIGHT_CUBE_STRAFE = 2000;

    private final double SPEED_MULTIPLIER = 0.7;

    public enum StartPosition {
        Crater,
        Depot
    }

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private ElapsedTime mDepotStateTime = new ElapsedTime();  // Time into current state
    private ElapsedTime mCraterStateTime = new ElapsedTime();  // Time into current state
    private State mCurrentMainState;
    private DepotState mCurrentDepotState;
    private CraterState mCurrentCraterState;

    private void newState(State newState){
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentMainState = newState;
    }

    private void newDepotState(DepotState newState) {
        mDepotStateTime.reset();
        mCurrentDepotState = newState;
    }

    private void newCraterState(CraterState newState) {
        mCraterStateTime.reset();
        mCurrentCraterState = newState;
    }

    public static enum State { // Ideally, these stay in order of how we use them
        STATE_INITIAL,
        STATE_DROP_FROM_LANDER,
        STATE_DETACH_LANDER,
        STATE_STRAFE_RIGHT,
        STATE_CLEAR_LANDER,
        STATE_LINEUP_MINERALS,
        STATE_HIT_MINERALS,
        STATE_DIFFERENTIATE_PATHS,
        STATE_IDLE,
        STATE_STOP
    }

    public enum DepotState {
        DS_INITIAL,
        DS_TURN_TO_DEPOT,
        DS_DRIVE_TO_DEPOT,
        DS_EJECT_MARKER,
        DS_TURN_TO_CRATER,
        DS_STRAFE_TO_WALL1,
        DS_CLEAR_WALL,
        DS_DRIVE_TO_CRATER1,
        DS_STRAFE_TO_WALL2,
        DS_DRIVE_TO_CRATER2,
        DS_STOP
    }

    public enum CraterState {
        CS_INITIAL,
        CS_HIT_CUBE,
        CS_UN_HIT_CUBE,
        CS_LINEUP_DEPOT,
        CS_TURN_TO_DEPOT,
        CS_DRIVE_TO_DEPOT1,
        CS_FLUSH_TO_WALL1,
        CS_CLEAR_WALL,
        CS_DRIVE_TO_DEPOT2,
        CS_EJECT_MARKER,
        CS_RETURN_TO_CRATER,
        CS_FLUSH_TO_WALL2,
        CS_APPROACH_CRATER,
        CS_STOP
    }

    //code
    @Override
    public void start() {
        resetStartTime();
        newState(State.STATE_INITIAL);
        newDepotState(DS_INITIAL);
        newCraterState(CS_INITIAL);
    }

    @Override
    public void init() {
        super.init();
        Pixy.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        Pixy.update();
        IsAuton = true;
        telemetry.addLine("Cube X Value: " + Pixy.getCubeX());
        telemetry.addLine("Cube Position: " + Pixy.getCubePosition());

        Arm.autoIntake(1);

        cubePosition = Pixy.getCubePosition();

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
        RunStateMachine();
        telemetry.addLine("STATE: " + mCurrentMainState)
                .addData("DEPOT: " , mCurrentDepotState)
                .addData("CRATER: ", mCurrentCraterState);
    }

    private void Crater() {
        switch (mCurrentCraterState) {
            case CS_INITIAL:
                NewMecanumDrive.stop();
                newCraterState(CS_HIT_CUBE);
                break;
            case CS_HIT_CUBE:
                speed = .75 * SPEED_MULTIPLIER;
                distance = 1000;
                if (NewMecanumDrive.right(speed, distance, 0)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_UN_HIT_CUBE);
                }
                break;

            case CS_UN_HIT_CUBE:
                speed = .75 * SPEED_MULTIPLIER;
                distance = 1000;
                if (NewMecanumDrive.left(speed, distance, 0)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_LINEUP_DEPOT);
                }
                break;

            case CS_LINEUP_DEPOT:
                speed = 0.5 * SPEED_MULTIPLIER;
                distance = 3000;
                if (NewMecanumDrive.forward(speed, distance, 0)) {
                    newCraterState(CS_TURN_TO_DEPOT);
                }
                break;

            case CS_TURN_TO_DEPOT:
                if (NewMecanumDrive.turn(45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_DRIVE_TO_DEPOT1);
                }
                break;

            case CS_DRIVE_TO_DEPOT1:
                speed = 0.5 * SPEED_MULTIPLIER;
                distance = 1400;
                if (NewMecanumDrive.forward(speed, distance, 45)) {
                    newCraterState(CS_FLUSH_TO_WALL1);
                }
                break;

            case CS_FLUSH_TO_WALL1:
                speed = .75 * SPEED_MULTIPLIER;
                distance = 500;
                if (NewMecanumDrive.right(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_CLEAR_WALL);
                }
                break;

            case CS_CLEAR_WALL:
                speed = .75 * SPEED_MULTIPLIER;
                distance = 300;
                if (NewMecanumDrive.left(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_DRIVE_TO_DEPOT2);
                }
                break;

            case CS_DRIVE_TO_DEPOT2:
                speed = .5 * SPEED_MULTIPLIER;
                distance = 500;
                if (NewMecanumDrive.backward(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_EJECT_MARKER);
                }
                break;

            case CS_EJECT_MARKER:
                Arm.autoIntake(1);
                if (mCraterStateTime.milliseconds() >= 3000){
                    Arm.autoIntake(0);
                    newCraterState(CS_RETURN_TO_CRATER);
                }
                break;

            case CS_RETURN_TO_CRATER:
                speed = 0.5 * SPEED_MULTIPLIER;
                distance = 2000;
                if (NewMecanumDrive.backward(speed, distance, 45)){
                    newCraterState(CS_FLUSH_TO_WALL2);
                }
                break;

            case CS_FLUSH_TO_WALL2:
                speed = .75 * SPEED_MULTIPLIER;
                distance = 500;
                if (NewMecanumDrive.right(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_APPROACH_CRATER);
                }
                break;

            case CS_APPROACH_CRATER:
                speed = 0.3 * SPEED_MULTIPLIER;
                distance = 400;
                if (NewMecanumDrive.backward(speed, distance, 45)){
                    newCraterState(CS_STOP);
                }
                break;

            case CS_STOP:
                newState(STATE_STOP);
        }
    }

    private void Depot() {
        switch (mCurrentDepotState) {
            case DS_INITIAL:
                NewMecanumDrive.stop();
                newDepotState(DS_TURN_TO_DEPOT);
                break;
            case DS_TURN_TO_DEPOT:
                speed = .0;
                if (NewMecanumDrive.turn(-depotAngle)){
                    NewMecanumDrive.stop();
                    newDepotState(DS_DRIVE_TO_DEPOT);
                }
                break;

            case DS_DRIVE_TO_DEPOT:
                switch (cubePosition) {
                    case UNKNOWN_CUBE:
                    case LEFT_CUBE:
                        distance = 2800;
                        heading = -LEFT_CUBE_ANGLE;
                        break;
                    case CENTER_CUBE:
                        distance = 2300;
                        heading = -CENTER_CUBE_ANGLE;
                        break;
                    case RIGHT_CUBE:
                        distance = 2500;
                        heading = -RIGHT_CUBE_ANGLE;
                        break;
                }
                speed = .5 * SPEED_MULTIPLIER;
                telemetry.addData("Distance", distance);
                if (NewMecanumDrive.forward(speed, distance, heading)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_EJECT_MARKER);
                }
                break;

            case DS_EJECT_MARKER:
                Arm.autoIntake(1);
                if (mDepotStateTime.milliseconds() >= 3000) {
                    Arm.autoIntake(0);
                    newDepotState(DS_TURN_TO_CRATER);
                }
                break;

            case DS_TURN_TO_CRATER:
                if (NewMecanumDrive.turn(-135)){
                    NewMecanumDrive.stop();
                    newDepotState(DS_STRAFE_TO_WALL1);
                }
                break;

            case DS_STRAFE_TO_WALL1:
                speed = .75 * SPEED_MULTIPLIER;
                switch (cubePosition){
                    case UNKNOWN_CUBE:
                    case LEFT_CUBE:
                        distance = LEFT_CUBE_STRAFE;
                    case CENTER_CUBE:
                        distance = CENTER_CUBE_STRAFE;
                    case RIGHT_CUBE:
                        distance = RIGHT_CUBE_STRAFE;
                }
                if (NewMecanumDrive.left(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_CLEAR_WALL);
                }
                break;

            case DS_CLEAR_WALL:
                speed = .75 * SPEED_MULTIPLIER;
                distance = 400;
                if (NewMecanumDrive.right(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_DRIVE_TO_CRATER1);
                }
                break;

            case DS_DRIVE_TO_CRATER1:
                speed = .5 * SPEED_MULTIPLIER;
                distance = 2500;
                if (NewMecanumDrive.backward(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_STRAFE_TO_WALL2);
                }
                break;

            case DS_STRAFE_TO_WALL2:
                speed = .75 * SPEED_MULTIPLIER;
                distance = 700;
                if (NewMecanumDrive.left(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_DRIVE_TO_CRATER2);
                }
                break;

            case DS_DRIVE_TO_CRATER2:
                speed = .3 * SPEED_MULTIPLIER;
                distance = 2500;
                if (NewMecanumDrive.backward(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_STOP);
                }
                break;

            case DS_STOP:
                newState(STATE_STOP);
                break;
        }
    }

    private void RunStateMachine() {
        switch (mCurrentMainState) {
            case STATE_INITIAL:
                Pixy.update();
                newState(STATE_DROP_FROM_LANDER);
                break;

            case STATE_DROP_FROM_LANDER:
                int dropDistance = 4150;
                Lift.setPosition(dropDistance);
                if (Math.abs(Lift.getTargetPos() - Lift.getCurrentPos()) <= 20) {
                    newState(STATE_DETACH_LANDER);
                }
                break;

            case STATE_DETACH_LANDER:
                speed = .3 * SPEED_MULTIPLIER;
                distance = 100;
                telemetry.addData("Distance", distance);

                if (NewMecanumDrive.backward(speed, distance, 0) || mStateTime.milliseconds() >= 2000){
                    NewMecanumDrive.stop();
                    newState(STATE_STRAFE_RIGHT);
                }
                break;

            case STATE_STRAFE_RIGHT:
                speed = .6 * SPEED_MULTIPLIER;
                distance = 175;
                if (NewMecanumDrive.right(speed, distance, 0)){
                    NewMecanumDrive.stop();
                    newState(STATE_CLEAR_LANDER);
                }
                break;

            case STATE_CLEAR_LANDER:
                speed = .3 * SPEED_MULTIPLIER;
                distance = 450;
                if (NewMecanumDrive.backward(speed, distance, 0)){
                    NewMecanumDrive.stop();
                    newState(STATE_LINEUP_MINERALS);
                }
                break;

            case STATE_LINEUP_MINERALS:
                speed = .7 * SPEED_MULTIPLIER;
                distance = 1500;
                if (NewMecanumDrive.right(speed, distance, 0)){
                    Lift.runToBottom();
                    NewMecanumDrive.stop();
                    newState(STATE_HIT_MINERALS);
                }

                break;

            case STATE_HIT_MINERALS:
                speed = .4 * SPEED_MULTIPLIER;
                switch (cubePosition) {
                    case UNKNOWN_CUBE:
                    case LEFT_CUBE:
                        if (lander == StartPosition.Depot)
                            distance = LEFT_CUBE_DISTANCE;
                        else
                            distance = LEFT_CUBE_CRATER_DISTANCE;
                        depotAngle = LEFT_CUBE_ANGLE;
                        if (NewMecanumDrive.forward(speed, distance, 0)){
                            NewMecanumDrive.stop();
                            newState(STATE_DIFFERENTIATE_PATHS);
                        }
                        break;

                    case CENTER_CUBE:
                        if (lander == StartPosition.Depot)
                            distance = CENTER_CUBE_DISTANCE;
                        else
                            distance = CENTER_CUBE_CRATER_DISTANCE;
                        depotAngle = CENTER_CUBE_ANGLE;
                        if (NewMecanumDrive.forward(speed, distance, 0)){
                            NewMecanumDrive.stop();
                            newState(STATE_DIFFERENTIATE_PATHS);
                        }
                        break;

                    case RIGHT_CUBE:
                        if (lander == StartPosition.Depot)
                            distance = RIGHT_CUBE_DISTANCE;
                        else
                            distance = RIGHT_CUBE_CRATER_DISTANCE;
                        depotAngle = RIGHT_CUBE_ANGLE;
                        if (NewMecanumDrive.backward(speed, distance, 0)){
                            NewMecanumDrive.stop();
                            newState(STATE_DIFFERENTIATE_PATHS);
                        }
                }
                break;

            case STATE_DIFFERENTIATE_PATHS:
                if (lander == StartPosition.Crater) {
                    Crater();
                    telemetry.addData("Position", lander);
                } else {
                    Depot();
                    telemetry.addData("Position", lander);
                }
                break;

            case STATE_STOP:
                NewMecanumDrive.stop();
                break;
        }
    }
}
