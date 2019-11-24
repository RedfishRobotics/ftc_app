package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name="Champs_TeleOp", group="Pushbot")
public class SlideLiftTest extends LinearOpMode {

    TeleOp_Library_Rover_Ruckus TeleOp = new TeleOp_Library_Rover_Ruckus();

//    double[] intakeLatch = new double[]{0, 1};
//    private int currentLatchIndex;
//    private boolean aPressedLast;

    @Override
    public void runOpMode() {

        TeleOp.init(hardwareMap);
//        this.currentLatchIndex = 0;
//        this.aPressedLast = false;
        waitForStart();

        while (opModeIsActive()){
//            final boolean yPressed = this.gamepad1.a;
//            if (yPressed && !this.aPressedLast)
//                this.currentLatchIndex = (this.currentLatchIndex + 1) % intakeLatch.length;
//            this.aPressedLast = yPressed;
            if(gamepad2.right_bumper){
                TeleOp.intakeLidOpen();
            }else{
                TeleOp.intakeLidClosed();
            }
            if(gamepad2.a){
                TeleOp.intakeIn();
            }
            if(gamepad2.b){
                TeleOp.intakeStop();
            }
            if(gamepad2.x){
                TeleOp.intakeOut();
            }
            if(gamepad2.dpad_up && TeleOp.magneticSwitchScoring.getState() == true && TeleOp.magneticSwitchStaging.getState() == true){
                TeleOp.scoringStandBy();
            }
            if(gamepad2.dpad_down){
                TeleOp.scoringDown();
            }
            if(gamepad1.right_bumper){
                TeleOp.scoringScore();
            }
            if(gamepad1.dpad_down){
                TeleOp.hangerDown();
            }
            if(gamepad1.dpad_up){
                TeleOp.hangerUp();
            }
            if(-gamepad2.right_stick_y > 0.2){
                TeleOp.right_Intake.setPower(0.8);
                TeleOp.left_Intake.setPower(0.8);
            }else{
                TeleOp.right_Intake.setPower(-gamepad2.right_stick_y);
                TeleOp.left_Intake.setPower(-gamepad2.right_stick_y);
            }
            if(-gamepad2.right_stick_y < -0.2){
                TeleOp.right_Intake.setPower(-0.8);
                TeleOp.left_Intake.setPower(-0.8);
            }else{
                TeleOp.right_Intake.setPower(-gamepad2.right_stick_y);
                TeleOp.left_Intake.setPower(-gamepad2.right_stick_y);
            }
//            TeleOp.boxArm.setPower(-gamepad2.left_stick_y);
            if(-gamepad2.left_stick_y < -0.2){
                TeleOp.boxArm.setPower(-0.5);
            }else{
                TeleOp.boxArm.setPower(-gamepad2.left_stick_y);
            }
            if(-gamepad2.left_stick_y > 0.2){
                TeleOp.boxArm.setPower(0.5);
            }else{
                TeleOp.boxArm.setPower(-gamepad2.left_stick_y);
            }
            if(gamepad2.right_trigger > 0.2){
                TeleOp.boxServo.setPower(-0.5);
            }
            else if(gamepad2.left_trigger > 0.2){
                TeleOp.boxServo.setPower(0.5);
            }else{
                TeleOp.boxServo.setPower(gamepad2.left_trigger);
            }
//            TeleOp.right_Intake.setPower(-gamepad2.right_stick_y);
//            TeleOp.left_Intake.setPower(-gamepad2.right_stick_y);
            if (gamepad1.left_trigger > 0.2) {
                TeleOp.leftFrontDrive.setPower(0.8);
                TeleOp.leftRearDrive.setPower(-0.8);
                TeleOp.rightFrontDrive.setPower(0.8);
                TeleOp.rightRearDrive.setPower(-0.8);
            } if (gamepad1.right_trigger > 0.2) {
                TeleOp.leftFrontDrive.setPower(-0.8);
                TeleOp.leftRearDrive.setPower(0.8);
                TeleOp.rightFrontDrive.setPower(-0.8);
                TeleOp.rightRearDrive.setPower(0.8);
            } else {
                TeleOp.leftFrontDrive.setPower(gamepad1.left_stick_y);
                TeleOp.leftRearDrive.setPower(gamepad1.left_stick_y);
                TeleOp.rightFrontDrive.setPower(gamepad1.right_stick_y);
                TeleOp.rightRearDrive.setPower(gamepad1.right_stick_y);
            }
            telemetry.addLine("Lift Arm: " + TeleOp.scoring_arm_lifter.getCurrentPosition());
            telemetry.addLine("Hanger: " +TeleOp.hanger.getCurrentPosition());
            telemetry.update();
        }
    }

}
