package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift;

public class OscarBaseOp extends OpMode {

    @Override
    public void init() {

        // Set up telemetry for custom classes
        OscarCommon.InitTelemetry(telemetry);

        // get all HardwareMap devices initialized
        Hardware.init(hardwareMap);

        // set up the drivetrain
        MecanumDrive.init();
        MecanumDrive.zeroEncoders();

        Arm.init();
        Arm.zeroEncoders();

        Lift.init();
        Lift.zeroEncoders();

        Gyro.init();
        Gyro.zero();
    }

    @Override
    public void loop() {
        Gyro.update();
    }
}
