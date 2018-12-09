package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AutoStates;
import org.firstinspires.ftc.teamcode.Base.OscarBaseOp;
import org.firstinspires.ftc.teamcode.Base.Pixy;

import static org.firstinspires.ftc.teamcode.AutoStates.State.*;
import static org.firstinspires.ftc.teamcode.Base.Hardware.MechanismMotors.elevator;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Oscar: AutoStates", group = "Oscar")
public class Autonomous2019 extends OscarBaseOp {

    private AutoStates.StartPosition lander = AutoStates.StartPosition.Depot;

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

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state
    private AutoStates.State mCurrentState;

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

    @Override
    public void start() {
        resetStartTime();
        newState(AutoStates.State.STATE_INITIAL);
    }

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void init_loop() {
        super.init_loop();
        autonEnabled = true;
        telemetry.addLine("Cube X Value: " + Pixy.cubeX());
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
            lander = AutoStates.StartPosition.Crater;
        if (gamepad1.dpad_down)
            lander = AutoStates.StartPosition.Depot;
    }

    public void loop() {
        super.loop();
        telemetry.addLine("STATE: " + mCurrentState);
        switch (mCurrentState) {
            case STATE_INITIAL:
                cubePosition = Pixy.getCubePosition();
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
                if (cubePosition == AutoStates.CubePosition.Left || cubePosition == AutoStates.CubePosition.Unknown){
                    distance = 2350;
                    depotAngle = 110;
                    strafeDistance = 600;
                    if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                        if (lander == AutoStates.StartPosition.Depot){
                            newState(STATE_TURN_TO_DEPOT);
                        } else
                            newState(STATE_TURN_TO_DEPOT);//STATE_HIT_CUBE
                        MecanumDrive(0, 0, 0, 0);
                    }
                }
                else if (cubePosition == AutoStates.CubePosition.Center){
                    distance = 1050;
                    depotAngle = 90;
                    strafeDistance = 1400;
                    if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                        if (lander == AutoStates.StartPosition.Depot){
                            newState(STATE_TURN_TO_DEPOT);
                        } else
                            newState(STATE_TURN_TO_DEPOT);//STATE_HIT_CUBE
                        MecanumDrive(0, 0, 0, 0);
                    }
                }
                else if (cubePosition == AutoStates.CubePosition.Right){
                    distance = 1;
                    depotAngle = 70;
                    strafeDistance = 2000;
                    if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                        if (lander == AutoStates.StartPosition.Depot){
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
                if (lander == AutoStates.StartPosition.Depot){
                    distance = 2000;
                    speed = .5;
                } else
                    speed = .25;
                distance = 1000;

                if (MecanumDrive(speed, backwardMove(speed), 0, -distance)){
                    if (lander == AutoStates.StartPosition.Depot){
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

}