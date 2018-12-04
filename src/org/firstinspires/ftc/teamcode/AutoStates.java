package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.teamcode.AutoStates.State.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: AutoStates", group = "Oscar")
public class AutoStates extends BaseOp {
    public int position, encoderTarget;
    public double drivePower;
    public double speed = 0.0;
    public double direction;
    public boolean dpadDownLastLoop = false;
    public boolean dpadUpLastLoop = false;
    public int stateCounter = 0;
    public int cubeSeenTime = 100000;
    public final double CUBECOLOR = 0.55;
    public static final String TAG = "Vuforia VuMark Sample";
    public boolean isRed = true;
    public boolean isScanning = false;
    public boolean seeingCube = false;
    final double SCALE_FACTOR = 255;
    public boolean cubeSeen = false;
    int distance;
    double depotAngle;
    int strafeDistance;
    
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    // sometimes it helps to multiply the raw RGB values with a scale factor to amplify/attentuate the measured values.
    
    
    public boolean isCubeHit(){
        return seeingCube && mStateTime.milliseconds() - cubeSeenTime > 500;
    }
    
    public enum StartPosition {
        Crater,
        Depot,
    }
    
    public enum CubePosition {
        Right,
        Left,
        Center,
        Unknown
    }

    private double leftVal = 1;
    private double centerVal = 75;
    private double rightVal = 160;

    private StartPosition lander = StartPosition.Depot;
    private CubePosition cubePosition;

    public enum State { // Ideally, these stay in order of how we use them
        STATE_INITIAL,
        STATE_DROP,
        STATE_DETACH_LANDER,
        STATE_STRAFE_RIGHT,
        STATE_CLEAR_LANDER,
        STATE_LINEUP,
        STATE_BACK,
        STATE_SCAN_MINERALS,
        STATE_HIT_CUBE,
        STATE_TURN_TO_DEPOT,
        STATE_DRIVE_TO_DEPOT,
        STATE_DROP_MARKER,
        STATE_TURN_TO_CRATER,
        STATE_STRAFE_TO_WALL,
        STATE_CLEAR_WALL,
        STATE_DRIVE_TO_CRATER,
        STATE_FLUSH_WALL,
        STATE_APPROACH_CRATER,
        STATE_TELEOP_INIT,
        STATE_STOP
    }

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private State mCurrentState;

    public CubePosition checkCube() {
        pixyData = pixyCam.read(0x51, 5);
        numObjects = 0xFF&pixyData[0];
        pixyX = 0xFF&pixyData[1];
        pixyY = 0xFF&pixyData[2];
        pixyWidth = 0xFF&pixyData[3];
        pixyHeight = 0xFF&pixyData[4];

        if (pixyX >= rightVal) return CubePosition.Right;
        else if (pixyX >= centerVal) return CubePosition.Center;
        else if (pixyX >= leftVal) return CubePosition.Left;
        else return CubePosition.Unknown;
    }

    @Override
    public void start() {
        resetStartTime();
        newState(State.STATE_INITIAL);
        isRed = (lander == StartPosition.Crater || lander == StartPosition.Depot);
    }

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void init_loop() {
        super.init_loop();
        autonEnabled = true;
        cubePosition = checkCube();
        telemetry.addLine("Cube Position: " + pixyX);
        telemetry.addLine("Position: " + (isRed ? "Red" : "Blue"));
        telemetry.addLine("CubePosition: " + cubePosition);
        if(gamepad1.b) // red button
            isRed = true;
        if(gamepad1.x) // blue button
            isRed = false;

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
                cubePosition = checkCube();
                newState(STATE_DROP);
                break;

            case STATE_DROP:
                elevatorTargetPosition = 2*2100;//2070
                elevator.setTargetPosition(elevatorTargetPosition);
                elevator.setPower(.5);
                if(Math.abs(elevatorTargetPosition - elevator.getCurrentPosition()) <= 20){
                   newState(STATE_DETACH_LANDER);
                }
                break;

              case STATE_DETACH_LANDER:
               speed = .3;
               if (MecanumDrive(speed, forwardMove(speed), 0, 100)){
                    newState(STATE_STRAFE_RIGHT);
                    MecanumDrive(0, 0, 0, 0);
               }
               break;

           case STATE_STRAFE_RIGHT:
                speed = .7;
                if (MecanumDrive(speed, rightMove(speed), 0, 200)){
                    MecanumDrive(0, 0, 0, 0);
                    newState(STATE_CLEAR_LANDER);
               }
               break;

            case STATE_CLEAR_LANDER:
                speed = .3;
                if (MecanumDrive(speed, forwardMove(speed), 0, 450)){
                    newState(STATE_LINEUP);
                    MecanumDrive(0, 0, 0, 0);
                }
                break;

            case STATE_LINEUP:
                speed = .7;
                if (MecanumDrive(speed, rightMove(speed), 0, 1400)){
                    elevatorTargetPosition = 0;
                    elevator.setTargetPosition(elevatorTargetPosition);
                    elevator.setPower(.5);
                    newState(STATE_BACK);
                    MecanumDrive(0, 0, 0, 0);
                }
                break;

            case STATE_BACK:
                speed = .25;
                if (MecanumDrive(speed, forwardMove(speed), 0, 900)){
                    newState(STATE_SCAN_MINERALS);
                    MecanumDrive(0, 0, 0, 0);
                }
                break;

           case STATE_SCAN_MINERALS:
                speed = .4;
                if (cubePosition == CubePosition.Left || cubePosition == CubePosition.Unknown){
                    distance = 2350;
                    depotAngle = 110;
                    strafeDistance = 600;
                    if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                        if (lander == StartPosition.Depot){
                            newState(STATE_TURN_TO_DEPOT);
                        } else
                            newState(STATE_TURN_TO_DEPOT);//STATE_HIT_CUBE
                        MecanumDrive(0, 0, 0, 0);
                    }
                }
                else if (cubePosition == CubePosition.Center){
                    distance = 1050;
                    depotAngle = 90;
                    strafeDistance = 1400;
                    if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                        if (lander == StartPosition.Depot){
                            newState(STATE_TURN_TO_DEPOT);
                        } else
                            newState(STATE_TURN_TO_DEPOT);//STATE_HIT_CUBE
                        MecanumDrive(0, 0, 0, 0);
                    }
                }
                else if (cubePosition == CubePosition.Right){
                    distance = 1;
                    depotAngle = 70;
                    strafeDistance = 2000;
                    if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                        if (lander == StartPosition.Depot){
                            newState(STATE_TURN_TO_DEPOT);
                        } else
                            newState(STATE_TURN_TO_DEPOT);;//STATE_HIT_CUBE
                        MecanumDrive(0, 0, 0, 0);
                    }
                }
                break;

            case STATE_HIT_CUBE:
                speed = .7;
                if (MecanumDrive(speed, rightMove(speed), 0, 400)){
                    MecanumDrive(0, 0, 0, 0);
                    newState(STATE_STOP);
                }
                break;

           case STATE_TURN_TO_DEPOT:
               speed = 0.0;
               targetHeading = -depotAngle;
               if (MecanumDrive(speed, 0, rotationComp(), 0)){
                    newState(STATE_DRIVE_TO_DEPOT);
                    MecanumDrive(0, 0, 0, 0);
                }
                break;

           case STATE_DRIVE_TO_DEPOT:
               if (lander == StartPosition.Depot){
                   distance = 2000;
                   speed = .5;
               } else
                   speed = .25;
                   distance = 1000;

               if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                   if (lander == StartPosition.Depot){
                       newState(STATE_DROP_MARKER);
                   } else
                       newState(STATE_STOP);
                    MecanumDrive(0, 0, 0, 0);

               }
               break;

            case STATE_DROP_MARKER:

                if (mStateTime.milliseconds() >= 3000){
                    intakeCollect.setPower(0.0);
                } else {
                    intakeCollect.setPower(-0.95);
                    newState(STATE_TURN_TO_CRATER);
                }
                break;

           case STATE_TURN_TO_CRATER:
               speed = .0;
               targetHeading = -135;
               if (MecanumDrive(speed, 0, rotationComp(), 0)){
                    newState(STATE_STRAFE_TO_WALL);
                    MecanumDrive(0,0,0,0);

               }
               break;

           case STATE_STRAFE_TO_WALL:
               speed = .5;
               if (MecanumDrive(speed, leftMove(speed), 0, -strafeDistance)){
                    intakeCollect.setPower(0.0);
                    newState(STATE_CLEAR_WALL);
                    MecanumDrive(0, 0, 0, 0);
               }
               break;

            case STATE_CLEAR_WALL:
                speed = .5;
                if (MecanumDrive(speed, rightMove(speed), 0, 400)){
                    intakeCollect.setPower(0.0);
                    newState(STATE_DRIVE_TO_CRATER);
                    MecanumDrive(0, 0, 0, 0);
                }
                break;

           case STATE_DRIVE_TO_CRATER:
               speed = .5;
               if (MecanumDrive(speed, forwardMove(speed), 0, 1000)){
                 MecanumDrive(0, 0, 0, 0);
                 newState(STATE_FLUSH_WALL);
               }
               break;

            case STATE_FLUSH_WALL:
                speed = .5;
                if (MecanumDrive(speed, leftMove(speed), 0, -600)){
                    intakeCollect.setPower(0.0);
                    newState(STATE_APPROACH_CRATER);
                    MecanumDrive(0, 0, 0, 0);
                }
                break;

           case STATE_APPROACH_CRATER:
               speed = .3;
               if (MecanumDrive(speed, forwardMove(speed), 0, 3000)){
                 MecanumDrive(0, 0, 0, 0);
                 newState(STATE_STOP);
               }
               break;


         /*  case STATE_TELEOP_INIT:
             MecanumDrive(0, 0, 0, 0);
             intakeArmExtend.setTargetPosition(-700);
             newState(STATE_STOP);
           break;
        */

        case STATE_STOP:
            MecanumDrive(0, 0, 0, 0);
            break;

        }
    }

    private void newState(State newState) {
        // Reset the state time, and then change to next state.
        mStateTime.reset();
        mCurrentState = newState;
        telemetry.addData("State", mCurrentState);
        stateCounter++;
    }

    public boolean gyroCloseEnough(double epsilon) {
        return Math.abs(currentGyroHeading - targetHeading) < epsilon;
    }
}
