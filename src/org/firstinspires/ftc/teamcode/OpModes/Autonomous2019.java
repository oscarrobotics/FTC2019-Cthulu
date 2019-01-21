package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Gamepad;
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

    private static Gamepad lastGamepad1 = new Gamepad();

    private double speed = 0.0;
    public int distance;
    public int heading;
    public double depotAngle;
    private StartPosition lander = StartPosition.Depot;
    public Pixy.CubePosition cubePosition = Pixy.CubePosition.LEFT_CUBE;
    public static int waitTime = 0;

    private int LEFT_CUBE_DISTANCE = 1500;
    private int LEFT_CUBE_CRATER_DISTANCE = 1500;
    private int LEFT_CUBE_ANGLE = 105;
    private int LEFT_CUBE_STRAFE = 600;

    private int CENTER_CUBE_DISTANCE = 300;
    private int CENTER_CUBE_CRATER_DISTANCE = 300;
    private int CENTER_CUBE_ANGLE = 85;
    private int CENTER_CUBE_STRAFE = 1000;

    private int RIGHT_CUBE_DISTANCE = 1100;
    private int RIGHT_CUBE_CRATER_DISTANCE = 850;
    private int RIGHT_CUBE_ANGLE = 65;
    private int RIGHT_CUBE_STRAFE = 1800;

    private final double SPEED_MULTIPLIER = 0.9;
    private final double DISTANCE_MULTIPLIER = 1.0;

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
        STATE_WAIT,
        STATE_DROP_FROM_LANDER,
        STATE_DETACH_LANDER,
        STATE_STRAFE_RIGHT,
        STATE_CLEAR_LANDER,
        STATE_LINEUP_MINERALS,
        STATE_HIT_MINERALS,
        STATE_DIFFERENTIATE_PATHS,
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
        DS_CLEAR_WALL2,
        DS_DRIVE_TO_CRATER2,
        DS_STRAFE_TO_WALL3,
        DS_APPROACH_CRATER,
        DS_STOP
    }

    public enum CraterState {
        CS_INITIAL,
        CS_HIT_CUBE,
        CS_UN_HIT_CUBE,
        CS_LINEUP_DEPOT,
        CS_TURN_TO_DEPOT,
        CS_FLUSH_TO_WALL1,
        CS_CLEAR_WALL1,
        CS_DRIVE_TO_DEPOT1,
        CS_FLUSH_TO_WALL2,
        CS_CLEAR_WALL2,
        CS_DRIVE_TO_DEPOT2,
        CS_EJECT_MARKER,
        CS_RETURN_TO_CRATER,
        CS_FLUSH_TO_WALL3,
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
        waitTime = 0;
    }

    @Override
    public void init_loop() {
        super.init_loop();
        Pixy.update();
        IsAuton = true;
        telemetry.addLine("Cube X Value: " + Pixy.getCubeX());
        telemetry.addLine("Cube Position: " + Pixy.getCubePosition());

        cubePosition = Pixy.getCubePosition();

        switch (lander) {
            case Crater:
                telemetry.addLine("Crater Side Auto");
                break;
            case Depot:
                telemetry.addLine("Depot Side Auto");
                break;
        }

        if (gamepad1.x)
            lander = StartPosition.Crater;
        if (gamepad1.y)
            lander = StartPosition.Depot;

        if (!lastGamepad1.dpad_up && gamepad1.dpad_up)
            waitTime += 250;
        else if (!lastGamepad1.dpad_down && gamepad1.dpad_down)
            waitTime -= 250;

        if (waitTime < 0)
            waitTime = 0;
        telemetry.addLine("Wait Time: " + ((double)waitTime/1000));

        try {
            lastGamepad1.copy(gamepad1);
        } catch(Exception ex) {}
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
                distance = (int) (800 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, 0)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_UN_HIT_CUBE);
                }
                break;

            case CS_UN_HIT_CUBE:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (1000 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.left(speed, distance, 0)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_LINEUP_DEPOT);
                }
                break;

            case CS_LINEUP_DEPOT:
                speed = 0.5 * SPEED_MULTIPLIER;
                switch (cubePosition) {
                    case UNKNOWN_CUBE:
                    case LEFT_CUBE:
                        distance = 1700;
                        break;

                    case CENTER_CUBE:
                        distance = 2500;
                        break;

                    case RIGHT_CUBE:
                        distance = 3200;
                        break;
                }
                distance = (int) (distance * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.forward(speed, distance, 0) || mCraterStateTime.milliseconds() >= 6000) {
                    newCraterState(CS_TURN_TO_DEPOT);
                }
                break;


            case CS_TURN_TO_DEPOT:
                if (NewMecanumDrive.turn(45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_FLUSH_TO_WALL1);
                }
                break;

            case CS_FLUSH_TO_WALL1:
                speed = .85 * SPEED_MULTIPLIER;
                distance = (int) (1200 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_CLEAR_WALL1);
                }
                break;

            case CS_CLEAR_WALL1:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (300 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.left(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_DRIVE_TO_DEPOT1);
                }
                break;

            case CS_DRIVE_TO_DEPOT1:
                speed = 0.5 * SPEED_MULTIPLIER;
                distance = (int) (1900 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.forward(speed, distance, 45)) {
                    newCraterState(CS_FLUSH_TO_WALL2);
                }
                break;

            case CS_FLUSH_TO_WALL2:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (600 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_CLEAR_WALL2);
                }
                break;

            case CS_CLEAR_WALL2:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (300 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.left(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_DRIVE_TO_DEPOT2);
                }
                break;

            case CS_DRIVE_TO_DEPOT2:
                speed = .5 * SPEED_MULTIPLIER;
                switch (cubePosition) {
                    case UNKNOWN_CUBE:
                    case LEFT_CUBE:
                        distance = 700;
                        break;

                    case CENTER_CUBE:
                        distance = 400;
                        break;

                    case RIGHT_CUBE:
                        distance = 300;
                        break;
                }
                distance = (int) (distance * DISTANCE_MULTIPLIER);

                if (NewMecanumDrive.forward(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_EJECT_MARKER);
                }
                break;

            case CS_EJECT_MARKER:
                Arm.autoIntake(-1);
                if (mCraterStateTime.milliseconds() >= 3000){
                    Arm.autoIntake(0);
                    newCraterState(CS_RETURN_TO_CRATER);
                }
                break;

            case CS_RETURN_TO_CRATER:
                speed = 0.5 * SPEED_MULTIPLIER;
                distance = (int) (3000 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.backward(speed, distance, 45)){
                    newCraterState(CS_FLUSH_TO_WALL3);
                }
                break;

            case CS_FLUSH_TO_WALL3:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (800 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, 45)) {
                    NewMecanumDrive.stop();
                    newCraterState(CS_APPROACH_CRATER);
                }
                break;

            case CS_APPROACH_CRATER:
                speed = 0.3 * SPEED_MULTIPLIER;
                distance = (int) (2400 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.backward(speed, distance, 40)){//45
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
                distance = (int) (distance * DISTANCE_MULTIPLIER);
                speed = .5 * SPEED_MULTIPLIER;
                if (NewMecanumDrive.forward(speed, distance, heading)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_EJECT_MARKER);
                }
                break;

            case DS_EJECT_MARKER:
                Arm.autoIntake(-1);
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
                distance = (int) (distance * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.left(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_CLEAR_WALL);
                }
                break;

            case DS_CLEAR_WALL:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (300 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_DRIVE_TO_CRATER1);
                }
                break;

            case DS_DRIVE_TO_CRATER1:
                speed = .5 * SPEED_MULTIPLIER;
                distance = (int) (700 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.backward(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_STRAFE_TO_WALL2);
                }
                break;

            case DS_STRAFE_TO_WALL2:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (600 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.left(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_CLEAR_WALL2);
                }
                break;

            case DS_CLEAR_WALL2:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (200 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_DRIVE_TO_CRATER2);
                }
                break;

            case DS_DRIVE_TO_CRATER2:
                speed = .5 * SPEED_MULTIPLIER;
                distance = (int) (2500 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.backward(speed, distance, -133)) {//-135
                    NewMecanumDrive.stop();
                    newDepotState(DS_STRAFE_TO_WALL3);
                }
                break;

            case DS_STRAFE_TO_WALL3:
                speed = .75 * SPEED_MULTIPLIER;
                distance = (int) (300 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.left(speed, distance, -135)) {
                    NewMecanumDrive.stop();
                    newDepotState(DS_APPROACH_CRATER);
                }
                break;

            case DS_APPROACH_CRATER:
                speed = .3 * SPEED_MULTIPLIER;
                distance = (int) (1000 * DISTANCE_MULTIPLIER);
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
                newState(STATE_WAIT);
                break;

            case STATE_WAIT:
                if (mStateTime.milliseconds() >= waitTime) {
                    newState(STATE_DROP_FROM_LANDER);
                }
                break;

            case STATE_DROP_FROM_LANDER:
                int dropDistance = 4175 ;
                Lift.setPosition(dropDistance);
                if (Math.abs(Lift.getTargetPos() - Lift.getCurrentPos()) <= 20) {
                    newState(STATE_DETACH_LANDER);
                }
                break;

            case STATE_DETACH_LANDER:
                speed = .3 * SPEED_MULTIPLIER;
                distance = (int) (100 * DISTANCE_MULTIPLIER);

                if (NewMecanumDrive.backward(speed, distance, 0) || mStateTime.milliseconds() >= 2000){
                    NewMecanumDrive.stop();
                    newState(STATE_STRAFE_RIGHT);
                }
                break;

            case STATE_STRAFE_RIGHT:
                speed = .6 * SPEED_MULTIPLIER;
                distance = (int) (175 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, 0)){
                    NewMecanumDrive.stop();
                    newState(STATE_CLEAR_LANDER);
                }
                break;

            case STATE_CLEAR_LANDER:
                speed = .3 * SPEED_MULTIPLIER;
                distance = (int) (450 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.backward(speed, distance, 0)){
                    NewMecanumDrive.stop();
                    newState(STATE_LINEUP_MINERALS);
                }
                break;

            case STATE_LINEUP_MINERALS:
                speed = .7 * SPEED_MULTIPLIER;
                if (lander == StartPosition.Depot)
                    distance = 1400;
                else
                    distance = 1400;
                distance = (int) (distance * DISTANCE_MULTIPLIER);
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
                        distance = (int) (distance * DISTANCE_MULTIPLIER);
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
                        distance = (int) (distance * DISTANCE_MULTIPLIER);
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
                        distance = (int) (distance * DISTANCE_MULTIPLIER);
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
