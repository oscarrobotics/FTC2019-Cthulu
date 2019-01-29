package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Base.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Base.Pixy;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift;

import static org.firstinspires.ftc.teamcode.OpModes.TestAutonomous.CraterState.*;
import static org.firstinspires.ftc.teamcode.OpModes.TestAutonomous.DepotState.*;
import static org.firstinspires.ftc.teamcode.OpModes.TestAutonomous.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TEST AutoStates", group = "Oscar")
public class TestAutonomous extends OscarBaseOp {

    private static Gamepad lastGamepad1 = new Gamepad();

    private double speed = 0.0;
    public int distance;
    public int heading;
    public int depotAngle, goldAngle;
    private StartPosition lander = StartPosition.Depot;
    public Pixy.CubePosition cubePosition = Pixy.CubePosition.LEFT_CUBE;
    public static int waitTime = 0;

    private int LEFT_CUBE_DISTANCE = 1500;
    private int LEFT_CUBE_CRATER_DISTANCE = 1500;
    private int LEFT_CUBE_ANGLE = 55;
    private int LEFT_CUBE_STRAFE = 600;

    private int CENTER_CUBE_DISTANCE = 300;
    private int CENTER_CUBE_CRATER_DISTANCE = 350;
    private int CENTER_CUBE_ANGLE = 90;
    private int CENTER_CUBE_STRAFE = 1000;

    private int RIGHT_CUBE_DISTANCE = 1100;
    private int RIGHT_CUBE_CRATER_DISTANCE = 850;
    private int RIGHT_CUBE_ANGLE = 125;
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
        STATE_TURN_TO_MINERALS,
        STATE_ZERO_ARM,
        STATE_TURN_TO_GOLD,
        STATE_DIFFERENTIATE_PATHS,
        STATE_STOP
    }

    public enum DepotState {
        DS_INITIAL,
        DS_PREPARE_ARM,
        DS_EJECT_MARKER,
        DS_RETURN_ARM,
        DS_CENTER_MINERALS,
        DS_TURN_TO_GOLD,
        DS_PREPARE_INTAKE,
        DS_INTAKE_GOLD,
        DS_BACK,
        DS_LINEUP_SCORE,
        DS_TURN_SCORE,
        DS_SCORE_ARM,
        DS_MOVE_SCORE,
        DS_PREPARE_MOVEMENT,
        DS_TURN_TO_CRATER,
        DS_DRIVE_TO_CRATER,
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
                speed = 0.25 * SPEED_MULTIPLIER;
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
                newDepotState(DS_PREPARE_ARM);
                break;

            case DS_PREPARE_ARM:
                if (Arm.moveArmY(-500, 0.5)) {
                    if (Arm.moveArmX(-3000, 0.5) && mDepotStateTime.milliseconds() > 2000) {
                        Arm.stop();
                        newDepotState(DS_EJECT_MARKER);
                    }
                }
                break;

            case DS_EJECT_MARKER:
                Arm.autoIntake(-1);
                if (mDepotStateTime.milliseconds() >= 2000) {
                    Arm.autoIntake(0);
                    newDepotState(DS_RETURN_ARM);
                }
                break;

            case DS_RETURN_ARM:
                if (Arm.moveArmX(-1200, 0.5)) {
                    if (NewMecanumDrive.backward(0.5, 400, -90)) {
                        Arm.stop();
                        NewMecanumDrive.stop();
                        newDepotState(DS_CENTER_MINERALS);
                    }
                }
                break;

            case DS_CENTER_MINERALS:
                speed = .5 * SPEED_MULTIPLIER;
                distance = (int) (100 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.left(speed, distance, -90)){
                    NewMecanumDrive.stop();
                    newDepotState(DS_TURN_TO_GOLD);
                }
                break;

            case DS_TURN_TO_GOLD:
                switch (cubePosition){
                    case UNKNOWN_CUBE:
                    case LEFT_CUBE:
                        goldAngle = LEFT_CUBE_ANGLE;
                        break;
                    case CENTER_CUBE:
                        goldAngle = CENTER_CUBE_ANGLE;
                        break;
                    case RIGHT_CUBE:
                        goldAngle = RIGHT_CUBE_ANGLE;
                        break;
                }
                if (NewMecanumDrive.turn(-goldAngle)){
                    NewMecanumDrive.stop();
                    newDepotState(DS_PREPARE_INTAKE);
                }
                break;

            case DS_PREPARE_INTAKE:
                if (Arm.zeroY()) {
                    Arm.stop();
                    newDepotState(DS_INTAKE_GOLD);
                }
                break;

            case DS_INTAKE_GOLD:
                Arm.autoIntake(1);
                speed = .2 * SPEED_MULTIPLIER;
                distance = (int) (200 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.forward(speed, distance, -goldAngle) && mDepotStateTime.milliseconds() > 2000){
                    NewMecanumDrive.stop();
                    newDepotState(DS_BACK);
                }
                break;

            case DS_BACK:
                if (NewMecanumDrive.backward(0.25, 200, -goldAngle)){
                    NewMecanumDrive.stop();
                    newDepotState(DS_LINEUP_SCORE);
                }

            case DS_LINEUP_SCORE:
                speed = .5 * SPEED_MULTIPLIER;
                distance = (int) (200 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.backward(speed, distance, -90)){
                    if (Arm.moveArmX(-1600, 0.5)){
                        Arm.autoIntake(0);
                        NewMecanumDrive.stop();
                        Arm.stop();
                        newDepotState(DS_SCORE_ARM);
                    }
                }
                break;

            case DS_SCORE_ARM:
                if (Arm.autoScoreGold() || mDepotStateTime.milliseconds() >= 3000){
                    Arm.stop();
                    newDepotState(DS_MOVE_SCORE);
                }
                break;

            case DS_MOVE_SCORE:
                speed = .2 * SPEED_MULTIPLIER;
                distance = (int) (600 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.backward(speed, distance, -90) || mDepotStateTime.milliseconds() >= 2000){
                    Arm.autoIntake(1);
                    NewMecanumDrive.stop();
                    newDepotState(DS_PREPARE_MOVEMENT);
                }
                break;

            case DS_PREPARE_MOVEMENT:
                if (Arm.return1() || mDepotStateTime.milliseconds() >= 3000){
                    Arm.stop();
                    if (NewMecanumDrive.forward(0.3, 400, 0)) {
                        NewMecanumDrive.stop();
                        Arm.autoIntake(0);
                        Arm.stop();
                        newDepotState(DS_DRIVE_TO_CRATER);
                    }
                }
                break;

            case DS_DRIVE_TO_CRATER:
                speed = .5 * SPEED_MULTIPLIER;
                distance = (int) (200 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.forward(speed, distance, 45)) {
                    if (Arm.return2()){
                        NewMecanumDrive.stop();
                        Arm.stop();
                        newDepotState(DS_STOP);
                    }

                }
                break;

            case DS_APPROACH_CRATER:
                speed = .25 * SPEED_MULTIPLIER;
                distance = (int) (2000 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.forward(speed, distance, 45)) {
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
                distance = (int) (300 * DISTANCE_MULTIPLIER);

                if (NewMecanumDrive.backward(speed, distance, 0) || mStateTime.milliseconds() >= 2000){
                    NewMecanumDrive.stop();
                    newState(STATE_STRAFE_RIGHT);
                }
                break;

            case STATE_STRAFE_RIGHT:
                speed = .6 * SPEED_MULTIPLIER;
                distance = (int) (1200 * DISTANCE_MULTIPLIER);
                if (NewMecanumDrive.right(speed, distance, 0)){
                    NewMecanumDrive.stop();
                    newState(STATE_TURN_TO_MINERALS);
                }
                break;


            case STATE_TURN_TO_MINERALS:
                if (NewMecanumDrive.turn(-90)){
                    NewMecanumDrive.stop();
                    newState(STATE_ZERO_ARM);
                }
                break;

            case STATE_ZERO_ARM:
                if (Arm.zeroY()){
                    Arm.stop();
                    newState(STATE_DIFFERENTIATE_PATHS);
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
