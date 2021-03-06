package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Base.*;
import org.firstinspires.ftc.teamcode.Mechanisms.*;

public class OscarBaseOp extends OpMode {

    public static boolean IsAuton = false;

    @Override
    public void init() {

        // Set up telemetry for custom classes
        OscarCommon.InitTelemetry(telemetry);

        // get all HardwareMap devices initialized
        Hardware.init(hardwareMap);

        // set up the drivetrain
        NewMecanumDrive.init();
        NewMecanumDrive.zeroEncoders();

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
