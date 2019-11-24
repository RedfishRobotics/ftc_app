package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOp_Library_Rover_Ruckus extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;
    public Servo intakeLid = null;
    public DcMotor intake = null;
    public CRServo boxArm = null;
    public DigitalChannel magneticSwitchScoring = null;
    public DigitalChannel magneticSwitchStaging = null;
    public DigitalChannel magneticSwitchDown = null;
    public Servo panServo = null;
    public CRServo boxServo = null;
    public CRServo right_Intake = null;
    public CRServo left_Intake = null;
    public DcMotor hanger = null;
    public DcMotor scoring_arm_lifter = null;

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
        hanger = hardwareMap.get(DcMotor.class, "Hanger");
        scoring_arm_lifter = hardwareMap.get(DcMotor.class, "Scoring_Arm_Lifter");
        boxArm = hardwareMap.get(CRServo.class, "Box_Arm");
        right_Intake = hardwareMap.get(CRServo.class, "Right_Intake");
        left_Intake = hardwareMap.get(CRServo.class, "Left_Intake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        magneticSwitchStaging = hardwareMap.get(DigitalChannel.class, "Stage_Hull_Effect");
        magneticSwitchDown = hardwareMap.get(DigitalChannel.class, "Down_Hull_Effect");
        magneticSwitchScoring = hardwareMap.get(DigitalChannel.class, "Scoring_Hull_Effect");
        boxServo = hardwareMap.get(CRServo.class, "Box_Servo");
        intakeLid = hardwareMap.get(Servo.class, "Intake_Lid");
        panServo = hardwareMap.get(Servo.class, "Pan_Servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        left_Intake.setDirection(CRServo.Direction.FORWARD);
        right_Intake.setDirection(CRServo.Direction.REVERSE);
        hanger.setDirection(DcMotor.Direction.REVERSE);
        scoring_arm_lifter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        intakeLid.setPosition(0.5);
        panServo.setPosition(0.5);

//        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        scoring_arm_lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void hangerUp(){
        Hanger(17000);
    }
    public void hangerHalf(){
        Hanger(8850);
    }
    public void hangerDown(){
        Hanger(10);
    }
    public void scoringStandBy(){
        ScoringArmLift(3200);
//        sleep(250);
//        if(magneticSwitchStaging.getState() == false){
//            boxServo.setPower(0.0);
//            boxArm.setPower(0.0);
//        }else{
//            boxServo.setPower(0.15);
//            boxArm.setPower(0.5);
//        }
        while(magneticSwitchStaging.getState() == true){
            boxArm.setPower(0.5);
            boxServo.setPower(0.15);
        }
        boxServo.setPower(0.0);
        boxArm.setPower(0.0);
    }
    public void scoringScore(){
        while(magneticSwitchScoring.getState() == true){
            boxArm.setPower(0.5);
            boxServo.setPower(-0.22);
        }
        boxArm.setPower(0.0);
        boxServo.setPower(0.0);
    }
    public void scoringDown(){
        ScoringArmLift(10);
//        sleep(250);
    }
    public void intakeIn(){
        intake.setPower(1.0);
    }
    public void intakeOut(){
        intake.setPower(-0.5);
    }
    public void intakeStop(){
        intake.setPower(0.0);
    }
    public void intakeLidOpen(){
        intakeLid.setPosition(0.5);
    }
    public void intakeLidClosed(){
        intakeLid.setPosition(0.96);
    }
    public void Hanger(int target) {
            //Sets the new target position for the glyph lifter
            // RightLiftMotor.setTargetPosition(target);
            hanger.setTargetPosition(target);
            //Turns on RUN_TO_POSITION
            // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //If statement that ask if the motor is busy
            if (hanger.isBusy()) {
                //If statement that checks if the motors current position is more then the target
                if (hanger.getCurrentPosition() > target) {
                    //If the current position is more than the target, set motor power to 40%
                    // RightLiftMotor.setPower(0.45);
                    hanger.setPower(0.65);
                }
                //If statement that checks if the motors current position is less then the target
                else if (hanger.getCurrentPosition() < target) {
                    //If the current position is more than the target, set motor power to 60%
                    // RightLiftMotor.setPower(0.5);
                    hanger.setPower(0.65);

                }
            }
        }
    public void ScoringArmLift(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        scoring_arm_lifter.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scoring_arm_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( scoring_arm_lifter.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( scoring_arm_lifter.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                scoring_arm_lifter.setPower(0.65);
            }
            //If statement that checks if the motors current position is less then the target
            else if( scoring_arm_lifter.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                scoring_arm_lifter.setPower(0.65);
            }
        }
    }
}
