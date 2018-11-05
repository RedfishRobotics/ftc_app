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

    private ElapsedTime runtime = new ElapsedTime();
    Servo SwervePod1 = null;
    Servo SwervePod2 = null;
    Servo SwervePod3 = null;
    Servo SwervePod4 = null;
    Servo ScoreFlipper = null;
    Servo cubeKnockRight = null;
    Servo cubeKnockLeft = null;
    DcMotor IntakeLiftRight = null;
    DcMotor IntakeLiftLeft = null;
    DcMotor SwervePod1motor = null;
    DcMotor SwervePod2motor = null;
    DcMotor SwervePod3motor = null;
    DcMotor SwervePod4motor = null;
    DcMotor IntakeMotor = null;
    DcMotor LiftMotor = null;

    private static final double[] Latch = new double[]{0, 1};
    private int currentLatchIndex;

    @Override
    public void runOpMode() {

        this.currentLatchIndex = 0;

        SwervePod1 = hardwareMap.get(Servo.class, "Swerve_Pod1");
        SwervePod2 = hardwareMap.get(Servo.class, "Swerve_Pod2");
        SwervePod3 = hardwareMap.get(Servo.class, "Swerve_Pod3");
        SwervePod4 = hardwareMap.get(Servo.class, "Swerve_Pod4");
        cubeKnockLeft = hardwareMap.get(Servo.class, "knockleft");
        cubeKnockRight = hardwareMap.get(Servo.class, "knockright");
        ScoreFlipper = hardwareMap.get(Servo.class, "ScoreFlipper");
//        IntakeSlide = hardwareMap.get(DcMotor.class, "IntakeSlide");

        SwervePod1motor = hardwareMap.get(DcMotor.class, "Swerve_Pod1motor");
        SwervePod2motor = hardwareMap.get(DcMotor.class, "Swerve_Pod2motor");
        SwervePod3motor = hardwareMap.get(DcMotor.class, "Swerve_Pod3motor");
        SwervePod4motor = hardwareMap.get(DcMotor.class, "Swerve_Pod4motor");
        IntakeLiftLeft = hardwareMap.get(DcMotor.class, "IntakeLiftLeft");
        IntakeLiftRight = hardwareMap.get(DcMotor.class, "IntakeLiftRight");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
        SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
        IntakeLiftRight.setDirection(DcMotor.Direction.REVERSE);
        IntakeLiftLeft.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        SwervePod1motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod2motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod3motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod4motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SwervePod1motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod2motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod3motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod4motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double SwervePod1Position = 0.5;
        double SwervePod2Position = 0.5;
        double SwervePod3Position = 0.5;
        double SwervePod4Position = 0.5;
        double CubeKnockLeftPosition = 0.92;
        double CubeKnockRightPosition = 0.13;
        double ScoreFlipperPosition = 0.2;
        ScoreFlipper.setPosition(ScoreFlipperPosition);
        SwervePod2.setPosition(SwervePod2Position);
        SwervePod1.setPosition(SwervePod1Position);
        SwervePod3.setPosition(SwervePod3Position);
        SwervePod4.setPosition(SwervePod4Position);
        cubeKnockRight.setPosition(CubeKnockRightPosition);
        cubeKnockLeft.setPosition(CubeKnockLeftPosition);
        //IntakeSlide.setPosition(IntakePosition);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        waitForStart();

        while (opModeIsActive()) {
            SwervePod1motor.setPower(-gamepad1.right_stick_y);
            SwervePod2motor.setPower(-gamepad1.right_stick_y);
            SwervePod3motor.setPower(-gamepad1.left_stick_y);
            SwervePod4motor.setPower(-gamepad1.left_stick_y);
            LiftMotor.setPower(-gamepad2.left_stick_y);
            IntakeLiftLeft.setPower(-gamepad2.right_stick_y);
            IntakeLiftRight.setPower(-gamepad2.right_stick_y);

            if (currentLatchIndex == 0) {
                SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
                SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
                SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
                SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
            } else if (currentLatchIndex == 1) {
                SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
                SwervePod2motor.setDirection(DcMotor.Direction.FORWARD);//changed
                SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
                SwervePod4motor.setDirection(DcMotor.Direction.REVERSE);//changed
            }
//            if(gamepad2.a){
//                liftTestRight(700);
//                liftTestLeft(700);
//            }
//            if(gamepad2.b){
//                liftTestRight(150);
//                liftTestLeft(150);
//            }
            if(gamepad1.dpad_up){
                IntakeMotor.setPower(0.75);
            }
            if(gamepad1.dpad_left){
                IntakeMotor.setPower(0.0);
            }
            if(gamepad1.dpad_down){
                IntakeMotor.setPower(-0.5);
            }
            if(gamepad1.dpad_right){
                IntakeMotor.setPower(0.25);
            }
            if (gamepad1.a) { //Middle
                SwervePod1.setPosition(0.5);
                SwervePod2.setPosition(0.5);
                SwervePod3.setPosition(0.5);
                SwervePod4.setPosition(0.5);
                currentLatchIndex = 0 % Latch.length;
            }
            if (gamepad1.x) {//turn
                SwervePod1.setPosition(0.8);
                SwervePod2.setPosition(0.2);
                SwervePod3.setPosition(0.8);
                SwervePod4.setPosition(0.2);
                currentLatchIndex = 1 % Latch.length;
            }
            if (gamepad1.y) {//45 degree to spin
                SwervePod1.setPosition(0.68);
                SwervePod2.setPosition(0.35);
                SwervePod3.setPosition(0.68);
                SwervePod4.setPosition(0.35);
                currentLatchIndex = 0 % Latch.length;
            }
            if (gamepad1.b) {//turn
                SwervePod1.setPosition(0.8);
                SwervePod2.setPosition(0.2);
                SwervePod3.setPosition(0.8);
                SwervePod4.setPosition(0.2);
                currentLatchIndex = 1 % Latch.length;
            }
            if (gamepad1.right_bumper) {//45 to the right
                SwervePod1.setPosition(0.35);
                SwervePod2.setPosition(0.35);
                SwervePod3.setPosition(0.35);
                SwervePod4.setPosition(0.35);
                currentLatchIndex = 0 % Latch.length;
            }
            if (gamepad1.left_bumper) {//45 to the left
                SwervePod1.setPosition(0.685);
                SwervePod2.setPosition(0.68);
                SwervePod3.setPosition(0.68);
                SwervePod4.setPosition(0.685);
                currentLatchIndex = 0 % Latch.length;
            }
            if(gamepad2.dpad_up){
                //scoreLift(-240);
            }
            if(gamepad2.dpad_down){
               // scoreLift(360);
            }
            if(gamepad2.dpad_left){
               // scoreLift(10);
            }
            if(gamepad2.right_bumper){
                ScoreFlipper.setPosition(0.2);
//                cubeKnockRight.setPosition(0.80);
//                cubeKnockLeft.setPosition(0.25);
            }
            if(gamepad2.left_bumper){
                ScoreFlipper.setPosition(0.7);
//                cubeKnockLeft.setPosition(0.92);
//                cubeKnockRight.setPosition(0.13);
            }
            if(gamepad2.dpad_right){

            }
            RobotLog.i("RF9958  –Runtime: " + runtime.seconds()
                    + "  RF9958 - lift encoder " + IntakeLiftRight.getCurrentPosition());
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

    public void liftTestLeft(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        IntakeLiftLeft.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( IntakeLiftLeft.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( IntakeLiftLeft.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                IntakeLiftLeft.setPower(0.25);
            }
            //If statement that checks if the motors current position is less then the target
            else if( IntakeLiftLeft.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                IntakeLiftLeft.setPower(0.25);
            }
        }
    }
    public void liftTestRight(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        IntakeLiftRight.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( IntakeLiftRight.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( IntakeLiftRight.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                IntakeLiftRight.setPower(0.25);
            }
            //If statement that checks if the motors current position is less then the target
            else if( IntakeLiftRight.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                IntakeLiftRight.setPower(0.25);
            }
        }
    }
}