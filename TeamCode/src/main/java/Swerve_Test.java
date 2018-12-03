/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Swerve_Library;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Swerve_Test", group="Pushbot")

public class Swerve_Test extends LinearOpMode {
    Swerve_Library swerveLibrary = new Swerve_Library();
    @Override
    public void runOpMode() {
        swerveLibrary.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drivetrainSmooth(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            swerveLibrary.intakeLift.setPower(-gamepad2.right_stick_y);
           //swerveLibrary.LiftMotor.setPower(-gamepad2.left_stick_y);
//            swerveLibrary.IntakeLiftLeft.setPower(-gamepad2.right_stick_y);
//            swerveLibrary.IntakeLiftRight.setPower(-gamepad2.right_stick_y);

            if(gamepad2.right_bumper){
                liftTestRight(0);
                //liftTestLeft(0);
            }
            if(gamepad2.left_bumper){
                liftTestRight(1100);
                //liftTestLeft(150);
            }

            if(gamepad1.dpad_up){
                swerveLibrary.IntakeMotor.setPower(-0.75);
            }
            if(gamepad1.dpad_left){
                swerveLibrary.IntakeMotor.setPower(0.0);
            }
            if(gamepad1.dpad_down){
                swerveLibrary.IntakeMotor.setPower(0.25);
            }
            if(gamepad1.dpad_right){
                swerveLibrary.IntakeMotor.setPower(-0.5);
            }
            if (gamepad1.a) { //Middle
                swerveLibrary.straightPosition();
            }
            if (gamepad1.x) {//turn
                swerveLibrary.turnPosition();
            }
            if (gamepad1.y) {//45 degree to spin
                swerveLibrary.spinPosition();
            }
            if (gamepad1.b) {//turn
               swerveLibrary.turnPosition();
            }
            if (gamepad1.right_bumper) {//45 to the right
                swerveLibrary.right45Position();
            }
            if (gamepad1.left_bumper) {//45 to the left
               swerveLibrary.left45Position();
            }
            if(gamepad2.dpad_up){
                //scoreLift(-240);
//                intakeLiftTest(10);
                swerveLibrary.WebcamPan.setPosition(1.0);
            }
            if(gamepad2.dpad_down){
               // scoreLift(360);
//                intakeLiftTest(300);
                swerveLibrary.WebcamPan.setPosition(0.15);
            }
            if(gamepad2.dpad_left){
               // scoreLift(10);
//                intakeLiftTest(615);
                swerveLibrary.WebcamPan.setPosition(0.66);
            }
            if(gamepad2.a){//down
                swerveLibrary.intakeRightLift.setPosition(0.0);
                swerveLibrary.intakeLeftLift.setPosition(1.0);
            }
            if(gamepad2.b){//middle
                swerveLibrary.intakeRightLift.setPosition(0.5);
                swerveLibrary.intakeLeftLift.setPosition(0.5);
            }
            if(gamepad2.x){//up
                swerveLibrary.intakeRightLift.setPosition(1.0);
                swerveLibrary.intakeLeftLift.setPosition(0.0);
            }
//            if(gamepad2.right_bumper){
//                swerveLibrary.ScoreFlipper.setPosition(0.2);
////                cubeKnockRight.setPosition(0.80);
////                cubeKnockLeft.setPosition(0.25);
//            }
//            if(gamepad2.left_bumper){
//                swerveLibrary.ScoreFlipper.setPosition(0.7);
////                cubeKnockLeft.setPosition(0.92);
////                cubeKnockRight.setPosition(0.13);
//            }
            if(gamepad2.dpad_right){

            }
            RobotLog.i("RF9958  –Runtime: " + swerveLibrary.runtime.seconds()
                    + "  RF9958 - lift encoder " + swerveLibrary.IntakeLiftRight.getCurrentPosition());
//            if(gamepad1.right_stick_x >= -0.1 || gamepad1.right_stick_x <= 0.1) {
//                SwervePod1.setPosition(gamepad1.right_stick_x + 1 * 0.5);
//            }
//            else {
//                SwervePod1.setPosition(0.47);
//            }
//         if(gamepad1.a){
//             SwervePod1.setPosition(1.0);
//         }
//         if (gamepad1.b){
//             SwervePod1.setPosition(0.0);
//         }
//         if (gamepad1.x){
//             SwervePod1.setPosition(0.5);
//         }
        }
    }

    public void intakeLiftTest(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        swerveLibrary.LiftMotor.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swerveLibrary.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( swerveLibrary.LiftMotor.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( swerveLibrary.LiftMotor.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                swerveLibrary.LiftMotor.setPower(0.10);
            }
            //If statement that checks if the motors current position is less then the target
            else if( swerveLibrary.LiftMotor.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                swerveLibrary.LiftMotor.setPower(0.2);
            }
        }
    }
    public void liftTestRight(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        swerveLibrary.IntakeLiftRight.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swerveLibrary.IntakeLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( swerveLibrary.IntakeLiftRight.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( swerveLibrary.IntakeLiftRight.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                swerveLibrary.IntakeLiftRight.setPower(0.45);
            }
            //If statement that checks if the motors current position is less then the target
            else if( swerveLibrary.IntakeLiftRight.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                swerveLibrary.IntakeLiftRight.setPower(0.45);
            }
        }
    }
    public void drivetrainSmooth(double leftPower, double rightPower){
        double smoothedLeftPower = leftPower *leftPower* leftPower;
        double smoothedRightPower = rightPower * rightPower* rightPower;
        swerveLibrary.SwervePod1motor.setPower(smoothedRightPower);
        swerveLibrary.SwervePod2motor.setPower(smoothedRightPower);
        swerveLibrary.SwervePod3motor.setPower(smoothedLeftPower);
        swerveLibrary.SwervePod4motor.setPower(smoothedLeftPower);
    }
}