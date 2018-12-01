/*
Copyright 2017 FIRST Tech Challenge Team 5494

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class BasicTeleOp extends BaseOp {
    //-2552
    /* Declare OpMode members. */
    private final int ARMYPOSITIONSCORE = -2750;
    private final int ARMEXTENDSCORE = -2800;
    private final int ARMYPOSITIONPARALLEL = -100;
    private final int ARMEXTENDPARALLEL = -1350;

    private final int ARMEXTENDMAX = -3000;
    private final int ARMVERTMAX = -2850;//-2900;

    private boolean autoEnabled = false;
    private boolean isXPressed = false;
    
    private int armYtargetPos = 0;
    private int armExtendTargetPos = 0;
    
    private boolean isBPressed = false;
    private double rightTrigger;
    private double leftTrigger;
    private boolean xPressedLast = false;
    private boolean bPressedLast = false;
    private boolean contSpin = false;
    private boolean leftTriggerPressed = false;
    private boolean verticalArmLimitWasPressed = false;
    private boolean wasArmLimetEverPressed = false;
    
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        super.init();
        //waitForStart();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        super.init_loop();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        autonEnabled = false;
        elevator.setPower(.5); 
        intakeArmVertical.setPower(0.5);
        intakeArmExtend.setPower(0.5);


    } 

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        super.loop();

        MecanumGamepadDrive();
        RunElevator();
        RunArm();
        DumpControl();
        RunLimit();
        telemetry.addLine("Arm Vertical target " + armYtargetPos);
        telemetry.addLine("Arm Vertical actual   " + intakeArmVertical.getCurrentPosition());
        telemetry.addLine("Arm Horizontal Target: " + armExtendTargetPos);
        telemetry.addLine("Arm Horizontal Actual: " + intakeArmExtend.getCurrentPosition());
        telemetry.addLine("Elevator Target: " + elevatorTargetPosition);
        telemetry.addLine("Elevator Actual: " + elevator.getCurrentPosition());

        //telemetry.addLine("Dump Servo " );
    }
    
    
    public void RunArm(){
          
        //Intake
        
        //tried toggle
        
        if(gamepad2.right_bumper && !leftTriggerPressed) {
            contSpin = !contSpin;
            leftTriggerPressed = true;
        }
        leftTriggerPressed = gamepad2.right_bumper;

        if (gamepad2.right_trigger >= 0.05){
            rightTrigger = gamepad2.right_trigger;
            intakeCollect.setPower(rightTrigger*rightTrigger);
        }else if (gamepad2.left_trigger > 0.05){
            intakeCollect.setPower(-gamepad2.left_trigger);
        }else {
            if(contSpin){
                intakeCollect.setPower(1.0);
            } else{
                intakeCollect.setPower(0.0);
            }
        }
        //Arm fine control
        if (gamepad2.right_stick_y > 0.2 || gamepad2.right_stick_y < -0.2){
            armExtendTargetPos += (int)(gamepad2.right_stick_y*20);
        }else {
            armExtendTargetPos = intakeArmExtend.getCurrentPosition();
        }
        
        if (gamepad2.left_stick_y > 0.2 || gamepad2.left_stick_y < -0.2){
            armYtargetPos += (int)(gamepad2.left_stick_y*20);
        }
        else if (Math.abs(armYtargetPos - intakeArmVertical.getCurrentPosition()) >= 50){
            armYtargetPos = intakeArmVertical.getCurrentPosition();
        }
        
        if (gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2){
            
        }else if (gamepad2.right_stick_x > 0.2 || gamepad2.right_stick_x < -0.2){
            rotStickX = gamepad2.right_stick_x;
        }
        //Arm buttons
        if (wasArmLimetEverPressed) {
            if (gamepad2.x) {
                armYtargetPos = ARMYPOSITIONSCORE;
                if (intakeArmVertical.getCurrentPosition() < -2000) {
                    armExtendTargetPos = ARMEXTENDSCORE;
                }

            } else if (gamepad2.b) {
                armYtargetPos = ARMYPOSITIONPARALLEL;
                armExtendTargetPos = ARMEXTENDPARALLEL;
            }
        }


        if (armYtargetPos < ARMVERTMAX){
            armYtargetPos = ARMVERTMAX;
        }
        if(armExtendTargetPos < ARMEXTENDMAX){
            armExtendTargetPos = ARMEXTENDMAX;
        }

        if (armYtargetPos >= 0 && wasArmLimetEverPressed){
            armYtargetPos = 0;
        }
        
        
        intakeArmExtend.setTargetPosition(armExtendTargetPos);
        
        intakeArmVertical.setTargetPosition(armYtargetPos);
    }
    
    public void DumpControl(){
        
        if(gamepad2.left_bumper){
            dumpServo.setPosition(dumpOpenPosition);
        
        } else {
            dumpServo.setPosition(dumpInit);
            
        }
    }
    
    public void RunElevator(){
        if(gamepad2.dpad_up)
            elevatorTargetPosition += 30;
        else if(gamepad2.dpad_down)
            elevatorTargetPosition -= 30;
        else if(gamepad2.y)
            elevatorTargetPosition = 2*2075;
        
        if(gamepad2.a){
            elevatorTargetPosition = 0;
            elevator.setPower(0.5);
            intakeArmVertical.setPower(0.0);
            intakeArmExtend.setPower(0.0);
        } else{
            elevator.setPower(0.5);
            intakeArmVertical.setPower(0.5);
            intakeArmExtend.setPower(0.5);
        }

        if(elevatorTargetPosition > 2*2200){
            elevatorTargetPosition = 2*2200;  
        }

        if(elevatorTargetPosition < 10){
            elevatorTargetPosition = 20;
        }


        elevator.setTargetPosition(elevatorTargetPosition);

    }
    
    public void RunLimit(){
        boolean limitIsPressed = !limitSwitch.getState();

        telemetry.addLine("Limit Switch: " + (limitIsPressed ? "Pressed" : "Not Pressed"));
        // if button is hit, and wasn't hit last time
        if (!verticalArmLimitWasPressed && limitIsPressed) {
            intakeArmVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeArmVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeArmVertical.setTargetPosition(0);
            armYtargetPos = 0;
            //verticalArmLimitWasPressed = true; // we were pressed
            wasArmLimetEverPressed = true;
        }
        // if button is hit and WAS hit last time
        else if (verticalArmLimitWasPressed && limitIsPressed) {
            // do NOT change encoder
            //verticalArmLimitWasPressed = false;
        }
        verticalArmLimitWasPressed = limitIsPressed;

    }
        
}
