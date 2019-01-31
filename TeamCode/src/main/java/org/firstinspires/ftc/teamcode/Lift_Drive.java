package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name="LiftDrive", group="Pushbot")
public class Lift_Drive extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
//    public DcMotor leftFrontDrive = null;
//    public DcMotor leftRearDrive = null;
//    public DcMotor rightFrontDrive = null;
//    public DcMotor rightRearDrive = null;
//    public DcMotor intakeMotor = null;
//    public DcMotor intakeSlideMotor = null;
    public Servo intakeLid = null;
    public CRServo intake = null;
    public Servo panServo = null;
    public CRServo leftSlide = null;
    public CRServo rightSlide = null;
    public DcMotor rightLifter = null;
    public DcMotor leftLifter = null;

    @Override
    public void runOpMode() throws InterruptedException {

//        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_FM");
//        leftRearDrive = hardwareMap.get(DcMotor.class, "Left_RM");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_FM");
//        rightRearDrive = hardwareMap.get(DcMotor.class, "Right_RM");
        leftSlide = hardwareMap.get(CRServo.class, "Left_Slide");
        rightSlide = hardwareMap.get(CRServo.class, "Right_Slide");
        rightLifter = hardwareMap.get(DcMotor.class, "Right_Lifter");
        leftLifter = hardwareMap.get(DcMotor.class, "Left_Lifter");
        panServo = hardwareMap.get(Servo.class, "Pan_Servo");
        intakeLid = hardwareMap.get(Servo.class, "Intake_Lid");
        intake = hardwareMap.get(CRServo.class, "Intake_Servo");
//        intakeMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
//        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Slide_Motor");

//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(CRServo.Direction.FORWARD);
        rightSlide.setDirection(CRServo.Direction.FORWARD);
        rightLifter.setDirection(DcMotor.Direction.FORWARD);
        leftLifter.setDirection(DcMotor.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//        intakeSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        panServo.setPosition(0.49);
        intakeLid.setPosition(0.35);
//        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()){
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

            if(gamepad1.a){
                intake.setPower(0.8);
            }
            if(gamepad1.b){
                intake.setPower(0.0);
            }
            if(gamepad1.x){
                intake.setPower(-0.4);
            }
            if(gamepad2.x){
//                panServo.setPosition(0.35);
                intakeLid.setPosition(0.35);
            }
            if(gamepad2.b){
//                panServo.setPosition(0.49);
                intakeLid.setPosition(0.55);

            }
            if(gamepad2.a){
//                panServo.setPosition(0.65);
                intakeLid.setPosition(0.9);

            }
            if(gamepad1.x){
                leftSlide.setPower(0);
                rightSlide.setPower(0);
            }
//            leftSlide.setPower(-gamepad1.left_stick_y);
            rightSlide.setPower(-gamepad1.left_stick_y);
            rightLifter.setPower(-gamepad2.right_stick_y);
            leftLifter.setPower(-gamepad2.right_stick_y);
//            intakeSlideMotor.setPower(-gamepad2.left_stick_y);
//            if (gamepad1.left_trigger > 0.2) {
//                leftFrontDrive.setPower(gamepad1.left_trigger);
//                leftRearDrive.setPower(-gamepad1.left_trigger);
//                rightFrontDrive.setPower(-gamepad1.left_trigger);
//                rightRearDrive.setPower(gamepad1.left_trigger);
//            } if (gamepad1.right_trigger > 0.2) {
//                leftFrontDrive.setPower(-gamepad1.right_trigger);
//                leftRearDrive.setPower(gamepad1.right_trigger);
//                rightFrontDrive.setPower(gamepad1.right_trigger);
//                rightRearDrive.setPower(-gamepad1.right_trigger);
//            } else {
//                leftFrontDrive.setPower(-gamepad1.left_stick_y);
//                leftRearDrive.setPower(-gamepad1.left_stick_y);
//                rightFrontDrive.setPower(-gamepad1.right_stick_y);
//                rightRearDrive.setPower(-gamepad1.right_stick_y);
//            }
            RobotLog.i("right lift position "+ rightLifter.getCurrentPosition());
            RobotLog.i("left lift position " + leftLifter.getCurrentPosition());
            telemetry.addLine("Left lift position: " + leftLifter.getCurrentPosition());
            telemetry.addLine("Right lift position: "+ rightLifter.getCurrentPosition());
            telemetry.update();

        }
    }
//public void liftTestRight(int target){
//    //Sets the new target position for the glyph lifter
//    // RightLiftMotor.setTargetPosition(target);
//    rightLifter.setTargetPosition(target);
//    //Turns on RUN_TO_POSITION
//    // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    //If statement that ask if the motor is busy
//    if( rightLifter.isBusy()){
//        //If statement that checks if the motors current position is more then the target
//        if( rightLifter.getCurrentPosition() > target){
//            //If the current position is more than the target, set motor power to 40%
//            // RightLiftMotor.setPower(0.45);
//            rightLifter.setPower(0.4);
//        }
//        //If statement that checks if the motors current position is less then the target
//        else if( rightLifter.getCurrentPosition() < target){
//            //If the current position is more than the target, set motor power to 60%
//            // RightLiftMotor.setPower(0.5);
//            rightLifter.setPower(0.15);
//
//        }
//    }
//}
//    public void liftTestLeft(int target){
//        //Sets the new target position for the glyph lifter
//        // RightLiftMotor.setTargetPosition(target);
//        leftLifter.setTargetPosition(target);
//        //Turns on RUN_TO_POSITION
//        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //If statement that ask if the motor is busy
//        if( leftLifter.isBusy()){
//            //If statement that checks if the motors current position is more then the target
//            if( leftLifter.getCurrentPosition() > target){
//                //If the current position is more than the target, set motor power to 40%
//                // RightLiftMotor.setPower(0.45);
//                leftLifter.setPower(0.4);
//            }
//            //If statement that checks if the motors current position is less then the target
//            else if( leftLifter.getCurrentPosition() < target){
//                //If the current position is more than the target, set motor power to 60%
//                // RightLiftMotor.setPower(0.5);
//                leftLifter.setPower(0.15);
//            }
//        }
//    }
}
