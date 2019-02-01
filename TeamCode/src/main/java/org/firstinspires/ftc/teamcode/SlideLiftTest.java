package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name="NewRobotTest", group="Pushbot")
public class SlideLiftTest extends LinearOpMode {

    TeleOp_Library_Rover_Ruckus TeleOp = new TeleOp_Library_Rover_Ruckus();

    @Override
    public void runOpMode() throws InterruptedException {

        TeleOp.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                TeleOp.intakeIn();
            }
            if(gamepad1.b){
                TeleOp.intakeStop();
            }
            if(gamepad1.x){
                TeleOp.intakeOut();
            }
            if(gamepad1.dpad_down){
                TeleOp.lifterTucked();
            }
            if(gamepad1.dpad_right){
                TeleOp.lifterScoring();
            }
            if(gamepad1.dpad_up){
                TeleOp.lifterDown();
            }
            if (gamepad1.left_trigger > 0.2) {
                TeleOp.leftFrontDrive.setPower(gamepad1.left_trigger);
                TeleOp.leftRearDrive.setPower(-gamepad1.left_trigger);
                TeleOp.rightFrontDrive.setPower(-gamepad1.left_trigger);
                TeleOp.rightRearDrive.setPower(gamepad1.left_trigger);
            } if (gamepad1.right_trigger > 0.2) {
                TeleOp.leftFrontDrive.setPower(-gamepad1.right_trigger);
                TeleOp.leftRearDrive.setPower(gamepad1.right_trigger);
                TeleOp.rightFrontDrive.setPower(gamepad1.right_trigger);
                TeleOp.rightRearDrive.setPower(-gamepad1.right_trigger);
            } else {
                TeleOp.leftFrontDrive.setPower(-gamepad1.left_stick_y);
                TeleOp.leftRearDrive.setPower(-gamepad1.left_stick_y);
                TeleOp.rightFrontDrive.setPower(-gamepad1.right_stick_y);
                TeleOp.rightRearDrive.setPower(-gamepad1.right_stick_y);
            }
            RobotLog.i("right lift position "+ TeleOp.rightLifter.getCurrentPosition());
            RobotLog.i("left lift position " + TeleOp.leftLifter.getCurrentPosition());
            telemetry.addLine("Left lift position: " + TeleOp.leftLifter.getCurrentPosition());
            telemetry.addLine("Right lift position: "+ TeleOp.rightLifter.getCurrentPosition());
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
