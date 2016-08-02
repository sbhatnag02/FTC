package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Medha on 12/19/15.
 * Updated by Ishaan 1/6/2016
 * Updated by Ishaan 3/14/2016
 * Updated by Ishaan 3/15/2016 -- all power to 0 at start
 * Updated by Sarthak 3/17/16 -- Implement Limit Switch on arm
 */
public class MasterTeleOp2 extends OpMode {
    //define all motors on the Chassis
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    // define motors for the arm
    DcMotor hangAngle;
    DcMotor hangExtension;
    // define servos
    Servo debrisBlock;
    AnalogInput debrisTouch;
    AnalogInput extensionTouch;
    AnalogInput angleTouch;

    DigitalChannel angle_forward;
    DigitalChannel extension_forward;

    DigitalChannel alliance;


    //set the toggle states for zip line and all clear movements
    ElapsedTime dropTime;

    //set motor powers
    final double forwardPower = .75;
    final double backwardPower = -.5;
    final double turnPower = .5;
    final double openArmPower = .5;
    final double closeArmPower = -.5;
    final double extendArmPower = 1;




    // Set Servo Positions for Flag Servos
    final int toggleWaitTime = 10;
    final double analogPressed = .2;

    //Define States for the Debris Servo state machine
    enum debrisServo {off, up, down, upWait};
    debrisServo debrisServoState;

    public void setDriveMotors(double leftPower, double rightPower){
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

    }

    @Override
    public void init() {
        //make hardware connections
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightFront = hardwareMap.dcMotor.get("right_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightBack = hardwareMap.dcMotor.get("right_back");

        hangAngle = hardwareMap.dcMotor.get("hang_angle");
        hangExtension = hardwareMap.dcMotor.get("hang_extension");

        // Connect to all the servos
        debrisBlock = hardwareMap.servo.get("debris_block");

        // These are limit switches for the arm and debris blocker
        debrisTouch = hardwareMap.analogInput.get("debris_touch");
        extensionTouch = hardwareMap.analogInput.get("extension_touch");
        angleTouch = hardwareMap.analogInput.get("angle_touch");

        //Digital Limit Switches for Angle Extension and Rack Extension
        angle_forward = hardwareMap.digitalChannel.get("anglef");
        angle_forward.setMode(DigitalChannelController.Mode.INPUT);
        extension_forward = hardwareMap.digitalChannel.get("extensionf");
        extension_forward.setMode(DigitalChannelController.Mode.INPUT);

        alliance = hardwareMap.digitalChannel.get("alliance");
        alliance.setMode(DigitalChannelController.Mode.OUTPUT);
        alliance.setMode(DigitalChannelController.Mode.INPUT);

        //reverse left drive motors and hang angle
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        hangExtension.setDirection(DcMotor.Direction.REVERSE);

        //reset all encoders that are needed
        hangAngle.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        hangExtension.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        hangAngle.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        hangExtension.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // set all motor power to 0
        setDriveMotors(0, 0);
        hangAngle.setPower(0);
        hangExtension.setPower(0);


        //set the start position and limits to variables
        final int hangAngleStart = hangAngle.getCurrentPosition();


        //Assign starting debris state
        debrisServoState = debrisServo.off;


        //Create timer
        dropTime = new ElapsedTime();
    }

    @Override
    public void loop() {
        //gamepad1 = forward/backward/turning movement
        //gamepad2 = arm movement

        try {
            //get values from gamepads
            //pushing stick all the way UP returns -1
            //so we have to negate the values
            float leftY = -gamepad1.left_stick_y;
            float rightY = -gamepad1.right_stick_y;

            // The first section is to control the chassis movement
            // The chassis movement uses gamepad1
            // a -- move forward with forward power
            // b -- move backwards with backwards power
            // Left and Right trigger for pivoting left or right
            // Otherwise tank mode for moving
            if (gamepad1.dpad_up) {
                //move forward straight with a
                setDriveMotors(forwardPower, forwardPower);

            } else if (gamepad1.dpad_down) {
                //move backwards straight with b
                setDriveMotors(backwardPower, backwardPower);

            } else if (gamepad1.dpad_left) {
                //pivot left with left trigger
                setDriveMotors(-turnPower, turnPower);

            } else if (gamepad1.dpad_right) {
                //pivot right with right trigger
                setDriveMotors(turnPower, -turnPower);
            } else {
                //tank mode
                setDriveMotors(leftY, rightY);

            }

            //set zipline position to correct zipline servo
            //depending on the switch


            //Debris Servo State Machine
            switch(debrisServoState){

                //don't move the servo
                //if x is pressed go to up
                //otherwise stop the motor
                case off:
                    if(gamepad1.x){
                        debrisServoState = debrisServo.up;
                    }else{
                        debrisBlock.setPosition(.5);
                    }
                    break;

                //safety is dpad left
                //if limit switch is pressed go to up wait
                //move servo up otherwise
                case up:
                    if(gamepad2.dpad_left){
                        debrisServoState = debrisServo.off;
                    }else if(debrisTouch.getValue() > 1000){
                        debrisServoState = debrisServo.upWait;
                    }else{
                        debrisBlock.setPosition(0);
                    }
                    break;

                //safety is dpad left
                //if the timer is equal to 1 go to off
                //otherwise move servo down
                case down:
                    if(gamepad2.dpad_left){
                        debrisServoState = debrisServo.off;
                    }else if (dropTime.time() < 1){
                        debrisBlock.setPosition(1);
                    }else{
                        debrisServoState = debrisServo.off;
                    }
                    break;

                //safety is dpad left
                //if x is pressed move down and reset timer
                //if the switch is not pressed move up
                //otherwise don't put any power on motor
                case upWait:
                    if(gamepad2.dpad_left){
                        debrisServoState = debrisServo.off;
                    }else if(gamepad1.x){
                        debrisServoState = debrisServo.down;
                        dropTime.reset();
                    }else if(debrisTouch.getValue() < 500){
                        debrisBlock.setPosition(0);
                    }else{
                        debrisBlock.setPosition(.5);
                    }
            }

            if (gamepad1.left_trigger > analogPressed) {
                //open arm with left trigger
                if (angle_forward.getState() == true){
                    hangAngle.setPower(0);
                }
                else{
                    hangAngle.setPower(openArmPower);
                }


            } else if (gamepad1.right_trigger > analogPressed) {
                //Check if arm is at the limit
                if(angleTouch.getValue() > 0){
                    hangAngle.setPower(0);
                    telemetry.addData("Your at the angle limit", angleTouch.getValue());
                }
                else {
                    //if switch is not pressed close arm
                    hangAngle.setPower(closeArmPower);
                }

            } else {
                //don't do anything if no triggers are pressed
                hangAngle.setPower(0);

            }

            //Extend the arm
            if (gamepad1.y) {
                if(extension_forward.getState() == true){
                  hangExtension.setPower(0);
                }
                else {
                    //extend arm with up
                    hangExtension.setPower(extendArmPower);
                }
            } else if (gamepad1.a) {
                //retract arm with down
                if (extensionTouch.getValue()>1000){
                    //check if the arm is touching the switch
                    hangExtension.setPower(0);
                    telemetry.addData("Your at the limit", hangExtension.getCurrentPosition());

                }
                else{
                    //retract the arm if the switch isn't pressed
                    hangExtension.setPower(-extendArmPower);
                }

            } else {
                //don't do anything if nothing is pressed
                hangExtension.setPower(0);
            }
            //All Clear -- Toggle with 2 states

            telemetry.addData("angle", hangAngle.getCurrentPosition());
            telemetry.addData("extension", hangExtension.getCurrentPosition());
            telemetry.addData("debris", debrisTouch.getValue());
            telemetry.addData("angle touch", angleTouch.getValue());
        } catch (Throwable t) {
            telemetry.addData("Exception in MasterTeleOp", t.getMessage());
            telemetry.addData("Angle", angleTouch.getValue());
            telemetry.addData("Extension", extensionTouch.getValue());
            telemetry.addData("Angle Forward", angle_forward.getState());
            telemetry.addData("Extension forward", extension_forward.getState());
        }
    } // end of loop
}