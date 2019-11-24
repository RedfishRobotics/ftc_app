package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name="Drive_For_GoBuilda", group="Pushbot")
public class Lift_Drive extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
        public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;
//    public DcMotor intakeMotor = null;
////    public DcMotor intakeSlideMotor = null;
//    public Servo intakeLid = null;
//    public DcMotor intake = null;
//    public CRServo boxArm = null;
//    public DigitalChannel magneticSwitchScoring = null;
//    public DigitalChannel magneticSwitchStaging = null;
//    public DigitalChannel magneticSwitchDown = null;
//    public DcMotor intakeSlide = null;
//    public DcMotor boxLift = null;
//    public CRServo flipperServo = null;
//    public CRServo boxServo = null;
////    public Servo panServo = null;
//    public CRServo right_Intake = null;
//    public CRServo left_Intake = null;
//    public Servo panServo = null;
//    public DcMotor hanger = null;
//    public DcMotor scoring_arm_lifter = null;
//    //public DcMotor rightLifter = null;
    //public DcMotor leftLifter = null;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_FM");
        leftRearDrive = hardwareMap.get(DcMotor.class, "Left_RM");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_FM");
        rightRearDrive = hardwareMap.get(DcMotor.class, "Right_RM");
        //  rightLifter = hardwareMap.get(DcMotor.class, "Right_Lifter");
        //leftLifter = hardwareMap.get(DcMotor.class, "Left_Lifter");
//        hanger = hardwareMap.get(DcMotor.class, "Hanger");
//        scoring_arm_lifter = hardwareMap.get(DcMotor.class, "Scoring_Arm_Lifter");
//        boxArm = hardwareMap.get(CRServo.class, "Box_Arm");
//        right_Intake = hardwareMap.get(CRServo.class, "Right_Intake");
//        left_Intake = hardwareMap.get(CRServo.class, "Left_Intake");
//        intakeSlide = hardwareMap.get(DcMotor.class, "Intake_Slider");
//        boxLift = hardwareMap.get(DcMotor.class, "Box_Lift");
//        intake = hardwareMap.get(DcMotor.class, "Intake");
//        magneticSwitchStaging = hardwareMap.get(DigitalChannel.class, "Stage_Hull_Effect");
//        magneticSwitchDown = hardwareMap.get(DigitalChannel.class, "Down_Hull_Effect");
//        magneticSwitchScoring = hardwareMap.get(DigitalChannel.class, "Scoring_Hull_Effect");
//        flipperServo = hardwareMap.get(CRServo.class, "Flip_Servo");
//        boxServo = hardwareMap.get(CRServo.class, "Box_Servo");
//        rightLifter = hardwareMap.get(Servo.class, "Right_Lifter");
//        leftLifter = hardwareMap.get(Servo.class, "Left_Lifter");
////        panServo = hardwareMap.get(Servo.class, "Pan_Servo");
//        intakeLid = hardwareMap.get(Servo.class, "Intake_Lid");
//        panServo = hardwareMap.get(Servo.class, "Pan_Servo");
//        intake = hardwareMap.get(CRServo.class, "Intake_Servo");
//        intakeMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
//        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Slide_Motor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        // rightLifter.setDirection(DcMotor.Direction.FORWARD);
        // leftLifter.setDirection(DcMotor.Direction.REVERSE);
//        left_Intake.setDirection(CRServo.Direction.FORWARD);
//        right_Intake.setDirection(CRServo.Direction.REVERSE);
//        hanger.setDirection(DcMotor.Direction.REVERSE);
//        scoring_arm_lifter.setDirection(DcMotor.Direction.REVERSE);

//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//        intakeSlide.setDirection(DcMotor.Direction.REVERSE);
//        boxLift.setDirection(DcMotor.Direction.REVERSE);
//        intake.setDirection(DcMotor.Direction.REVERSE);

//        rightLifter.setPosition(0.95);
//        leftLifter.setPosition(0.05);
//        panServo.setPosition(0.49);
//        intakeLid.setPosition(0.5);
//        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        scoring_arm_lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        boxLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
//            scoring_arm_lifter.setPower(-gamepad1.left_stick_y);
//            hanger.setPower(-gamepad1.right_stick_y);
//            boxArm.setPower(-gamepad2.left_stick_y);
//            boxServo.setPower(-gamepad1.right_stick_y);
//            right_Intake.setPower(-gamepad2.right_stick_y);
//            left_Intake.setPower(-gamepad2.right_stick_y);
//            if(-gamepad2.right_stick_y > 0.2){
//                right_Intake.setPower(0.8);
//                left_Intake.setPower(0.8);
//            }else{
//                right_Intake.setPower(-gamepad2.right_stick_y);
//                left_Intake.setPower(-gamepad2.right_stick_y);
//            }
//            if(-gamepad2.right_stick_y < -0.2){
//                right_Intake.setPower(-0.8);
//                left_Intake.setPower(-0.8);
//            }else{
//                right_Intake.setPower(-gamepad2.right_stick_y);
//                left_Intake.setPower(-gamepad2.right_stick_y);
//            }

//            if(gamepad1.a){
//                Hanger(10);
//            }
//            if(gamepad1.b){
//                Hanger(8850);
//            }
//            if(gamepad1.x){
//                Hanger(17700);
//            }
//            if(gamepad2.a){
//                panServo.setPosition(0.5);
//            }
//            if(gamepad2.b) {
//                panServo.setPosition(0.6);
//            }
//            if(gamepad1.dpad_up){
//                while(magneticSwitchStaging.getState() == true){
//                    boxArm.setPower(0.5);
//                    boxServo.setPower(0.15);
//                }
//                    boxServo.setPower(0.0);
//                    boxArm.setPower(0.0);
//            }
//            if(gamepad1.dpad_down){
//                while(magneticSwitchDown.getState() == true){
//                    boxArm.setPower(-0.5);
//                    boxServo.setPower(-0.2);
//                }
//                    boxServo.setPower(0.0);
//                    boxArm.setPower(0.0);
//            }
//            if(gamepad1.dpad_right){
//                while(magneticSwitchScoring.getState() == true){
//                    boxArm.setPower(0.5);
//                    boxServo.setPower(0.2);
//                }
//                    boxArm.setPower(0.0);
//                    boxServo.setPower(0.0);
//            }
//            if(gamepad2.x){
//                panServo.setPosition(0.4);
//            }
//            if(gamepad2.dpad_right){
//                intake.setPower(0.5);
//            }
//            if(gamepad2.dpad_down){
//                intake.setPower(0.0);
//            }
//            if(gamepad2.dpad_up){
//                intake.setPower(-1.0);
//            }
//            if(gamepad2.right_bumper){
//                intakeLid.setPosition(0.5);
//            }
//            if(gamepad2.left_bumper){
//                intakeLid.setPosition(0.96);
//            }
//            if(gamepad1.a){
//                intakeMotor.setPower(0.4);
//            }
//            if(gamepad1.b){
//                intakeMotor.setPower(0.0);
//            }
//            if(gamepad1.dpad_down){
//                liftTestLeft(0);
//                liftTestRight(0);
//            }
//            if(gamepad1.dpad_right){
//                liftTestLeft(1750);
//                liftTestRight(1650);
//            }
//            if(gamepad1.dpad_up){
//                liftTestLeft(775);
//                liftTestRight(550);
//            }

//            if(gamepad1.a){
//                intake.setPower(0.8);
//            }
//            if(gamepad1.b){
//                intake.setPower(0.0);
//            }
//            if(gamepad1.x){
//                intake.setPower(-0.4);
//            }
//            if(gamepad2.x){
////                panServo.setPosition(0.35);
//                intakeLid.setPosition(0.35);
//            }
//            if(gamepad2.b){
////                panServo.setPosition(0.49);
//                intakeLid.setPosition(0.55);
//
//            }
//            if(gamepad2.a){
////                panServo.setPosition(0.65);
//                intakeLid.setPosition(0.9);
//
//            }
//            if(gamepad1.a){
//                intakeSlide.setPower(0.75);
//            }
//            if(gamepad1.x){
//                intakeSlide.setPower(-0.2);
//            }
//            if(gamepad1.b){
//                intakeSlide.setPower(0.1);
//            }
//            if(gamepad1.right_bumper) {
//                    intake.setPower(0.75);
//                }
//             else{
//                intake.setPower(0.0);
//            }
//            if(gamepad1.left_bumper){
//                intake.setPower(0.0);
//            }
//            if(gamepad2.left_bumper){
//                intaketoBox();
//            }
//            if(gamepad2.dpad_down){
//                boxInitFlip();
//            }
//            if(gamepad2.a){
//                rightLifter.setPosition(0.3);
//                leftLifter.setPosition(0.7);
//                sleep(500);
//                rightLifter.setPosition(0.01);
//                leftLifter.setPosition(0.99);
//            }
//            if(gamepad2.x){
//                rightLifter.setPosition(0.7);
//                leftLifter.setPosition(0.3);
//            }
//            if(gamepad2.b){
//                rightLifter.setPosition(0.85);
//                leftLifter.setPosition(0.15);
//            }
//            if(gamepad1.x){
//                SlideTest(10);
//            }
//            if(gamepad1.b){
//                SlideTest(50);
//            }
//            if(gamepad1.a){
//                SlideTest(250);
//            }
//            if(gamepad1.y){
//                SlideTest(550);
//            }
//            if(gamepad1.dpad_up){
////                BoxLiftTest(2200);
////                sleep(1000);
//                while (magneticSwitch.getState() == true){
//                    boxArm.setPower(0.75);
////                    boxServo.setPower(-0.15);
//                }
//                    boxArm.setPower(0);
////                    boxServo.setPower(0);
//
//            }
//            if(gamepad1.dpad_down){
//                BoxLiftTest(10);
//            }
//            boxServo.setPower(-gamepad2.left_stick_y/2);
//            flipperServo.setPower(-gamepad2.right_stick_y/2);
//            boxLift.setPower(-gamepad1.left_stick_y);
//            intakeSlide.setPower(-gamepad1.left_stick_y);
//            intakeSlide.setPower(-gamepad1.left_stick_y);
            //   rightLifter.setPower(-gamepad2.right_stick_y);
            // leftLifter.setPower(-gamepad2.right_stick_y);
//            intakeSlideMotor.setPower(-gamepad2.left_stick_y);
            if (gamepad1.left_trigger > 0.2) {
                leftFrontDrive.setPower(gamepad1.left_trigger);
                leftRearDrive.setPower(-gamepad1.left_trigger);
                rightFrontDrive.setPower(-gamepad1.left_trigger);
                rightRearDrive.setPower(gamepad1.left_trigger);
            } if (gamepad1.right_trigger > 0.2) {
                leftFrontDrive.setPower(-gamepad1.right_trigger);
                leftRearDrive.setPower(gamepad1.right_trigger);
                rightFrontDrive.setPower(gamepad1.right_trigger);
                rightRearDrive.setPower(-gamepad1.right_trigger);
            } else {
                leftFrontDrive.setPower(-gamepad1.left_stick_y);
                leftRearDrive.setPower(-gamepad1.left_stick_y);
                rightFrontDrive.setPower(-gamepad1.right_stick_y);
                rightRearDrive.setPower(-gamepad1.right_stick_y);
            }
//            RobotLog.i("right lift position "+ rightLifter.getCurrentPosition());
//            RobotLog.i("left lift position " + leftLifter.getCurrentPosition());
//            telemetry.addLine("Left lift position: " + leftLifter.getCurrentPosition());
//            telemetry.addLine("Right lift position: "+ rightLifter.getCurrentPosition());
//            telemetry.addLine("Intake: " + intake.getCurrentPosition());
//            telemetry.addLine("Scoring Arm Lifter: " + scoring_arm_lifter.getCurrentPosition());
//            telemetry.addLine("Hanger: " + hanger.getCurrentPosition());
            telemetry.update();

        }
    }

//    public void Hanger(int target) {
//        //Sets the new target position for the glyph lifter
//        // RightLiftMotor.setTargetPosition(target);
//        hanger.setTargetPosition(target);
//        //Turns on RUN_TO_POSITION
//        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //If statement that ask if the motor is busy
//        if (hanger.isBusy()) {
//            //If statement that checks if the motors current position is more then the target
//            if (hanger.getCurrentPosition() > target) {
//                //If the current position is more than the target, set motor power to 40%
//                // RightLiftMotor.setPower(0.45);
//                hanger.setPower(0.65);
//            }
//            //If statement that checks if the motors current position is less then the target
//            else if (hanger.getCurrentPosition() < target) {
//                //If the current position is more than the target, set motor power to 60%
//                // RightLiftMotor.setPower(0.5);
//                hanger.setPower(0.65);
//
//            }
//        }
//    }
//    public void ScoringArmLift(int target){
//        //Sets the new target position for the glyph lifter
//        // RightLiftMotor.setTargetPosition(target);
//        scoring_arm_lifter.setTargetPosition(target);
//        //Turns on RUN_TO_POSITION
//        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        scoring_arm_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //If statement that ask if the motor is busy
//        if( scoring_arm_lifter.isBusy()){
//            //If statement that checks if the motors current position is more then the target
//            if( scoring_arm_lifter.getCurrentPosition() > target){
//                //If the current position is more than the target, set motor power to 40%
//                // RightLiftMotor.setPower(0.45);
//                scoring_arm_lifter.setPower(0.65);
//            }
//            //If statement that checks if the motors current position is less then the target
//            else if( scoring_arm_lifter.getCurrentPosition() < target){
//                //If the current position is more than the target, set motor power to 60%
//                // RightLiftMotor.setPower(0.5);
//                scoring_arm_lifter.setPower(0.65);
//            }
//        }
//    }
//    public void intake(double power){
//        while(gamepad1.left_bumper) {
//            intake.setTargetPosition(intake.getCurrentPosition() + 144);
//            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            intake.setPower(power);
//        }
//    }
//    public void SlideTest(int target){
//        //Sets the new target position for the glyph lifter
//        // RightLiftMotor.setTargetPosition(target);
//        intakeSlide.setTargetPosition(target);
//        //Turns on RUN_TO_POSITION
//        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //If statement that ask if the motor is busy
//        if( intakeSlide.isBusy()){
//            //If statement that checks if the motors current position is more then the target
//            if( intakeSlide.getCurrentPosition() > target){
//                //If the current position is more than the target, set motor power to 40%
//                // RightLiftMotor.setPower(0.45);
//                intakeSlide.setPower(0.4);
//            }
//            //If statement that checks if the motors current position is less then the target
//            else if( intakeSlide.getCurrentPosition() < target){
//                //If the current position is more than the target, set motor power to 60%
//                // RightLiftMotor.setPower(0.5);
//                intakeSlide.setPower(0.4);
//            }
//        }
//    }
//    public void boxInitFlip(){
//        rightLifter.setPosition(0.3);
//        leftLifter.setPosition(0.7);
//        sleep(500);
//        flipperServo.setPower(0.5);
//        sleep(1000);
//        flipperServo.setPower(0);
//        boxServo.setPower(0.5);
//        sleep(1000);
//        boxServo.setPower(0);
//        flipperServo.setPower(-0.5);
//        sleep(1000);
//        flipperServo.setPower(0);
//    }
//    public void intaketoBox(){
//        SlideTest(10);
//        sleep(1000);
//        rightLifter.setPosition(0.7);
//        leftLifter.setPosition(0.3);
//        sleep(1500);
//        rightLifter.setPosition(0.3);
//        leftLifter.setPosition(0.7);
//        sleep(500);
//        BoxLiftTest(2200);
//        sleep(1000);
//        while (magneticSwitch.getState() == true){
//            flipperServo.setPower(0.75);
//            boxServo.setPower(-0.15);
//        }
//        flipperServo.setPower(0);
//        boxServo.setPower(0);
//    }
        }
//    }
//}