package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
        Gyro.init();
    }

    @Override
    public void loop() {
        Gyro.update();
    }
}
