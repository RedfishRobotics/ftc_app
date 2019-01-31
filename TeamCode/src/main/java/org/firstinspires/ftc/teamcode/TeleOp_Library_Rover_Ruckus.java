package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOp_Library_Rover_Ruckus extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public CRServo intake = null;
    public DcMotor rightRearDrive = null;
    public DcMotor intakeMotor = null;
    public DcMotor intakeSlideMotor = null;
    public DcMotor rightLifter = null;
    public DcMotor leftLifter = null;

    HardwareMap hardwareMap       =  null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // set references for config

        //relicPivot.setPosition(relicPivotPosition);

        // Wait for the game to start (driver presses PL
    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap  = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_FM");
        leftRearDrive = hardwareMap.get(DcMotor.class, "Left_RM");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_FM");
        rightRearDrive = hardwareMap.get(DcMotor.class, "Right_RM");
        rightLifter = hardwareMap.get(DcMotor.class, "Right_Lifter");
        leftLifter = hardwareMap.get(DcMotor.class, "Left_Lifter");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Slide_Motor");
        intake = hardwareMap.get(CRServo.class, "Intake_Servo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightLifter.setDirection(DcMotor.Direction.FORWARD);
        leftLifter.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void intakeStop(){
        intake.setPower(0.0);
    }
    public void intakeIn(){
        intake.setPower(0.8);
    }
    public void intakeOut(){
        intake.setPower(-0.4);
    }
    public void lifterTucked(){
        liftTestLeft(0);
        liftTestRight(0);
    }
    public void lifterScoring(){
        liftTestLeft(775);
        liftTestRight(550);
    }
    public void lifterDown(){
        liftTestLeft(1750);
        liftTestRight(1650);
    }
    public void liftTestRight(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        rightLifter.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( rightLifter.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( rightLifter.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                rightLifter.setPower(0.4);
            }
            //If statement that checks if the motors current position is less then the target
            else if( rightLifter.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                rightLifter.setPower(0.15);

            }
        }
    }
    public void liftTestLeft(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        leftLifter.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( leftLifter.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( leftLifter.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                leftLifter.setPower(0.4);
            }
            //If statement that checks if the motors current position is less then the target
            else if( leftLifter.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                leftLifter.setPower(0.15);
            }
        }
    }


}
