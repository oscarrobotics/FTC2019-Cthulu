package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Base.MecanumDrive;
import org.firstinspires.ftc.teamcode.Base.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp2019", group="Iterative Opmode")

public class TeleOp2019 extends OscarBaseOp {

    private static Gamepad lastGamepad1 = new Gamepad();
    private static Gamepad lastGamepad2 = new Gamepad();

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

        NewMecanumDrive.teleopControl(gamepad1, lastGamepad1);
        Lift.teleopControl(gamepad2);
        Arm.teleopControl(gamepad2);

        telemetry.addLine("Arm Vertical target " + Arm.getVerticalTargetPos());
        telemetry.addLine("Arm Vertical actual   " + Arm.getVerticalPos());
        telemetry.addLine("Arm Horizontal Target: " + Arm.getHorizontalPos());
        telemetry.addLine("Arm Horizontal Actual: " + Arm.getHorizontalPos());
        telemetry.addLine("Elevator Target: " + Lift.getTargetPos());
        telemetry.addLine("Elevator Actual: " + Lift.getTargetPos());
        //telemetry.addLine("Dump Servo " );


        try {
            lastGamepad1.copy(gamepad1);
            lastGamepad2.copy(gamepad2);
        } catch(Exception ex) {}
    }
}
