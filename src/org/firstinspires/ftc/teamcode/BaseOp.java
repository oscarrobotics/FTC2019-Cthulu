package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.Locale;

public class BaseOp extends OpMode {

  /**
   * To Do List:
   *
   *
   *
   **/



    // imus
    BNO055IMU imu;


    // Motors
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor leftBack;
    public DcMotor elevator;
    public DcMotor intakeCollect;
    public DcMotor intakeArmVertical;
    public DcMotor intakeArmExtend;

    //Servos
    public Servo dumpServo;


    // Gyro
    BNO055IMU.Parameters gyroParams;
    Orientation angles;
    Acceleration gravity;

    // Sensors
    public I2cDeviceSynch pixyCam;
    DigitalChannel limitSwitch;

    // Variables
    public boolean isLeftHandDrive = false;
    double lastKnownRotJoy = 0.0;
    double targetHeading = 0.0;
    double currentGyroHeading = 0.0;
    double rotStickX = 0;
    double driveStickY = 0;
    double driveStickX = 0;
    boolean autonEnabled = false;
    double testHead = 72;
    public double hitCube = 0.2;
    public double scanPosition = 0.45;
    public double colorArmInit = 1;
    public double dumpOpenPosition = 0.1;
    public double dumpInit = 0.95;
    public int elevatorTargetPosition = 0;

    //Pixy
    public double pixyX, pixyY, pixyWidth, pixyHeight, numObjects;
    public byte[] pixyData;

    // Mecanum Variables
    double speed = 0;
    // double lastSpeed = 0;
    double direction = 0;
    double rotation = 0;
    int target = 0;
    int targetDestination = 0;




    public double forwardMove(double speed) {
        return Math.atan2(speed, 0) - Math.PI / 4;
    }

    public double backwardMove(double speed) {
        return Math.atan2(-speed, 0) - Math.PI / 4;
    }

    public double rightMove(double speed) {
        return Math.atan2(0, -speed) - Math.PI / 4;
    }

    public double leftMove(double speed) {
        return Math.atan2(0, speed) - Math.PI / 4;
    }

    public void gyroInit(){

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
         gyroParams = new BNO055IMU.Parameters();
         gyroParams.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
         gyroParams.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         gyroParams.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
         gyroParams.loggingEnabled      = true;
         gyroParams.loggingTag          = "IMU";
     }

    public void zeroEncoders() {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public double leftAndBack(double speed) {return Math.atan2((0.25*speed), speed) - Math.PI / 4;}

    public void init() {
        telemetry.addLine("Start of init");



        // Motor block
        rightFront = hardwareMap.dcMotor.get("rightFront");

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack = hardwareMap.dcMotor.get("rightBack");

        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.dcMotor.get("leftFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.dcMotor.get("leftBack");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator = hardwareMap.dcMotor.get("elevator");

        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeCollect = hardwareMap.dcMotor.get("intakeCollect");

        intakeCollect.setDirection(DcMotor.Direction.FORWARD);
        intakeCollect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//FLOAT

        intakeArmVertical = hardwareMap.dcMotor.get("intakeArmVertical");

        intakeArmVertical.setDirection(DcMotor.Direction.FORWARD);
        intakeArmVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeArmExtend = hardwareMap.dcMotor.get("intakeArmExtend");

        intakeArmExtend.setDirection(DcMotor.Direction.FORWARD);
        intakeArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        zeroEncoders();

        //servos

        dumpServo = hardwareMap.servo.get("dumpServo");
        dumpServo.setPosition(dumpInit);


        //sensors
        pixyCam = hardwareMap.i2cDeviceSynch.get("pixy");
        pixyCam.engage();


        // Gyro stuff
        ////NEGATIVE = clockwise
        gyroInit();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParams);

        telemetry.addLine("initialized");
    }

    @Override
    public void init_loop() { // runs after pressing INIT and loops until START pressed
        super.init_loop();

    }

    public void loop() { // constantly running code

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //telemetry.addData("5","Gyro heading:",angles.firstAngle);
        currentGyroHeading = angles.firstAngle;
        telemetry.update();
        telemetry.addLine("Elevator Position: " + elevator.getCurrentPosition());
    }

    protected void MecanumGamepadDrive() {
        target = 0;
        double maxIncrement = 100;
        boolean rotationCorrection = true;

        if (isLeftHandDrive) {
            rotStickX = gamepad1.right_stick_x;
            driveStickX = -gamepad1.left_stick_x;
            driveStickY = gamepad1.left_stick_y;
        } else {
            rotStickX = gamepad1.left_stick_x;
            driveStickX = gamepad1.right_stick_x;
            driveStickY = gamepad1.right_stick_y;
        }
        rotStickX = Math.pow(-rotStickX, 3);


        if (rotationCorrection) {
            if (rotStickX == 0 && lastKnownRotJoy != 0.0) {
                targetHeading = currentGyroHeading;
            }

            lastKnownRotJoy = rotStickX;

            if (rotStickX != 0) {
                targetHeading = (currentGyroHeading + (maxIncrement * rotStickX)) % 360;
            }
        } else {
            rotation = rotStickX;
        }



        // DPad mDrive
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (gamepad1.dpad_down) { // backwards
                speed = gamepad1.right_bumper ? .5 : 0.3;
                direction = Math.atan2(speed, 0) - Math.PI / 4;
            } else if (gamepad1.dpad_left) { // left
                speed = 0.75;
                direction = Math.atan2(0, speed) - Math.PI / 4;
            } else if (gamepad1.dpad_up) { // forwards
                speed = gamepad1.right_bumper ? .5 : .3;
                direction = Math.atan2(-speed, 0) - Math.PI / 4;
            } else { // right
                speed = 0.75;
                direction = Math.atan2(0, -speed) - Math.PI / 4;
            }
        }
        // Joystick mDrive
        else {
            speed = Math.hypot(driveStickX, driveStickY);
            direction = Math.atan2(driveStickY, -driveStickX) - Math.PI / 4;
        }

        // if(Math.abs(speed - lastSpeed) >= 0.02)
        //     speed = speed > lastSpeed ? lastSpeed + 0.02 : lastSpeed - 0.02;

        MecanumDrive(speed, direction, rotation, target, rotationCorrection);
        // lastSpeed = speed;
    }

    protected boolean MecanumDrive(double speed, double direction, double rotation, int target) {

        boolean rotationCorrection = true;
        return MecanumDrive(speed, direction, rotation, target, rotationCorrection);

    }

    protected boolean MecanumDrive(double speed, double direction, double rotation, int target, boolean rotationCorrection) {

        if (rotation == 0) {
            rotation = rotationComp();
        }


        if (target != 0 && targetDestination == 0) {
            targetDestination = leftBack.getCurrentPosition() + target;
        }

        telemetry.addLine()
                .addData("TargetHeading", targetHeading)
                .addData("ActualHeading", currentGyroHeading);
        telemetry.addLine()
                .addData("rotation", rotation);

        telemetry.addLine()
        .addData("Actual", leftBack.getCurrentPosition())
        .addData("Dest", targetDestination);


        final double v1 = speed * Math.cos(direction) + rotation;
        final double v2 = speed * Math.sin(direction) - rotation;
        final double v3 = speed * Math.sin(direction) + rotation;
        final double v4 = speed * Math.cos(direction) - rotation;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftBack.setPower(v3);
        rightBack.setPower(v4);

        if ((targetDestination != 0 && Math.abs(leftBack.getCurrentPosition() - targetDestination) < 100)
            || (direction == 0) && (Math.abs(targetHeading-currentGyroHeading) <=3) ) {
            targetDestination = 0;
            return true;
        } else {
            return false;
        }
    }

    protected double rotationComp(boolean isTurning) {
        double rotation = 0.0;
        double gyro = currentGyroHeading;
        double target = targetHeading;
        double posError = gyro - target;
        double epsilon = autonEnabled?3:10;
        double minSpeed = isTurning?.45:.2; // was 0.12
        double maxSpeed = 1;

        if (Math.abs(posError) > 180) {
            posError = -360 * Math.signum(posError) + posError;
        }
        if (Math.abs(posError) > epsilon) {
            rotation = minSpeed + (Math.abs(posError) / 180) * (maxSpeed - minSpeed);
            rotation = rotation * Math.signum(posError);
        }
        return -rotation;
    }

    protected double rotationComp()
    {
        return rotationComp(false);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
