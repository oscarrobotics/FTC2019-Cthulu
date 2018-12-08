package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AutoStates;
import org.firstinspires.ftc.teamcode.Base.OscarBaseOp;
import org.firstinspires.ftc.teamcode.Base.Pixy;

public class Autonomous2019 extends OscarBaseOp {

    private AutoStates.StartPosition lander = AutoStates.StartPosition.Depot;

    public enum StartPosition {
        Crater,
        Depot,
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


}