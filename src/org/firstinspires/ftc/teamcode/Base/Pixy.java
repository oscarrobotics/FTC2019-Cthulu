package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class Pixy extends OscarCommon {

    private static I2cDeviceSynch _pixyCam;

    private static final int leftBound = 1;
    private static final int rightBound = 75;
    private static final int centerBound = 160;

    public static void init(){
        _pixyCam = Hardware.Sensors.pixyCam;
        _pixyCam.engage();
    }

    public static void update() {
        byte[] i2cData = _pixyCam.read(0x51, 5);
        PixyData.numObjects = 0xFF&i2cData[0];
        PixyData.x = 0xFF&i2cData[1];
        PixyData.y = 0xFF&i2cData[2];
        PixyData.width = 0xFF&i2cData[3];
        PixyData.height = 0xFF&i2cData[4];

        if (PixyData.x >= rightBound) PixyData.cubePosition = CubePosition.RIGHT_CUBE;
        else if (PixyData.x >= centerBound) PixyData.cubePosition = CubePosition.CENTER_CUBE;
        else if (PixyData.x >= leftBound) PixyData.cubePosition = CubePosition.LEFT_CUBE;
        else PixyData.cubePosition = CubePosition.UNKNOWN_CUBE;
        _telemetry.addLine("CubePos: " + PixyData.cubePosition);
        // defaults to Unknown
    }

    public static int getCubeX(){
        return PixyData.x;
    }

    public static int getCubeY(){
        return PixyData.y;
    }

    public static int getCubeWidth(){
        return PixyData.width;
    }

    public static int getCubeHeight(){
        return PixyData.height;
    }

    public static CubePosition getCubePosition(){
        return PixyData.cubePosition;
    }

    public enum CubePosition {
        RIGHT_CUBE,
        LEFT_CUBE,
        CENTER_CUBE,
        UNKNOWN_CUBE
    }

    private static class PixyData {
        public static int numObjects;
        public static int x;
        public static int y;
        public static int width;
        public static int height;
        public static CubePosition cubePosition;
    }
}
