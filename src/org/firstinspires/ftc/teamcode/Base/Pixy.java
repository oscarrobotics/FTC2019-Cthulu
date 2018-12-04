package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class Pixy extends OscarCommon {

    private static I2cDeviceSynch _pixyCam;

    private static final int rightBound = 160;
    private static final int leftBound = 0;
    private static final int centerBound = 0;

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

        if (PixyData.x >= rightBound) PixyData.cubePosition = CubePosition.Right;
        else if (PixyData.x >= centerBound) PixyData.cubePosition = CubePosition.Center;
        else if (PixyData.x >= leftBound) PixyData.cubePosition = CubePosition.Left;
        // defaults to Unknown
    }

    public static int cubeX(){
        return PixyData.x;
    }

    public static int cubeY(){
        return PixyData.y;
    }

    public static int cubeWidth(){
        return PixyData.width;
    }

    public static int cubeHeight(){
        return PixyData.height;
    }

    public static CubePosition getCubePosition(){
        return PixyData.cubePosition;
    }

    public enum CubePosition {
        Right,
        Left,
        Center,
        Unknown
    }

    private static class PixyData {
        public static int numObjects;
        public static int x;
        public static int y;
        public static int width;
        public static int height;
        public static CubePosition cubePosition = CubePosition.Unknown;
    }
}
