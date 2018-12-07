package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.Base.Hardware;
import org.firstinspires.ftc.teamcode.Base.MecanumDrive;
import org.firstinspires.ftc.teamcode.Base.OscarBaseOp;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp2019", group="Iterative Opmode")

public class TeleOp2019 extends OscarBaseOp {

    private boolean autoEnabled = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        super.init();
        //waitForStart();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        autoEnabled = false;
//        Hardware.MechanismMotors.elevator.setPower(.5);
//        Hardware.MechanismMotors.intakeArmVertical.setPower(0.5);
//        Hardware.MechanismMotors.intakeArmExtend.setPower(0.5);
    }

    @Override
    public void loop() {
        super.loop();

        MecanumDrive.teleopDrive(gamepad1);
        Lift.teleopControl(gamepad2);
        Arm.teleopControl(gamepad2);
        DumpControl();
        telemetry.addLine("Arm Vertical target " + Arm.getVerticalTargetPos());
        telemetry.addLine("Arm Vertical actual   " + Arm.getVerticalPos());
        telemetry.addLine("Arm Horizontal Target: " + Arm.getHorizontalPos());
        telemetry.addLine("Arm Horizontal Actual: " + Arm.getHorizontalPos());
        telemetry.addLine("Elevator Target: " + Lift.getTargetPos());
        telemetry.addLine("Elevator Actual: " + Lift.getTargetPos());
        //telemetry.addLine("Dump Servo " );
    }

    public void DumpControl(){
        Arm.dumpMineral(gamepad2.left_bumper);
    }
}
