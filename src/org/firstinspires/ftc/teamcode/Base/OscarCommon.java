package org.firstinspires.ftc.teamcode.Base;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OscarCommon {
    protected static Telemetry _telemetry;

    public static void InitTelemetry(Telemetry telemetry) {
        _telemetry = telemetry;
    }
}
