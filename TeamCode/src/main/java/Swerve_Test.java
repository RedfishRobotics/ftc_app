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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    Servo Flipper1 = null;
    Servo Flipper2 = null;
    DcMotor IntakeLift = null;
    DcMotor SwervePod1motor = null;
    DcMotor SwervePod2motor = null;
    DcMotor SwervePod3motor = null;
    DcMotor SwervePod4motor = null;
    DcMotor IntakeMotor = null;
    DcMotor LiftMotor = null;
//    DcMotor LiftMotorRight = null;
//    DcMotor LiftMotorLeft = null;


    /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
    double SwerveLeft;
    double SwerveRight;
    double Swerve_MIN_LEFT = 0.5;
    double Swerve_MAX_LEFT = 1.0;
    double Swerve_MIN_RIGHT = 0.0;
    double Swerve_MAX_RIGHT = 0.5;

    double IntakePosition = 0.065;
    double IntakePositionClipped;
    double Intake_MIN = 0.065;
    double Intake_MAX = 0.23;
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    private static final double[] Latch = new double[]{0, 1};
    private int currentLatchIndex;

    @Override
    public void runOpMode() {

        this.currentLatchIndex = 0;

        SwervePod1 = hardwareMap.get(Servo.class, "Swerve_Pod1");
        SwervePod2 = hardwareMap.get(Servo.class, "Swerve_Pod2");
        SwervePod3 = hardwareMap.get(Servo.class, "Swerve_Pod3");
        SwervePod4 = hardwareMap.get(Servo.class, "Swerve_Pod4");
        Flipper1 = hardwareMap.get(Servo.class, "Flipper_Right");
        Flipper2 = hardwareMap.get(Servo.class, "Flipper_Left");
//        IntakeSlide = hardwareMap.get(DcMotor.class, "IntakeSlide");

        SwervePod1motor = hardwareMap.get(DcMotor.class, "Swerve_Pod1motor");
        SwervePod2motor = hardwareMap.get(DcMotor.class, "Swerve_Pod2motor");
        SwervePod3motor = hardwareMap.get(DcMotor.class, "Swerve_Pod3motor");
        SwervePod4motor = hardwareMap.get(DcMotor.class, "Swerve_Pod4motor");
        IntakeLift = hardwareMap.get(DcMotor.class, "IntakeLift");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
//        LiftMotorLeft = hardwareMap.get(DcMotor.class, "LiftMotorLeft");
//        LiftMotorRight = hardwareMap.get(DcMotor.class, "LiftMotorRight");

        SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
        SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
        IntakeLift.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
//        LiftMotorRight.setDirection(DcMotor.Direction.REVERSE);
//        LiftMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        SwervePod1motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod2motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod3motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod4motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        IntakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LiftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LiftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SwervePod1motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod2motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod3motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod4motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        IntakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LiftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        LiftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double SwervePod1Position = 0.5;
        double SwervePod2Position = 0.5;
        double SwervePod3Position = 0.5;
        double SwervePod4Position = 0.5;
        double Flipper1Position = 0.1;
        double Flipper2Position = 0.9;
        SwervePod2.setPosition(SwervePod2Position);
        SwervePod1.setPosition(SwervePod1Position);
        SwervePod3.setPosition(SwervePod3Position);
        SwervePod4.setPosition(SwervePod4Position);
        Flipper1.setPosition(Flipper1Position);
        Flipper2.setPosition(Flipper2Position);
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
//            LiftMotorLeft.setPower(-gamepad2.right_stick_y);
//            LiftMotorRight.setPower(-gamepad2.right_stick_y);
//            IntakeLift.setPower(-gamepad2.left_stick_y/2);
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
            if(gamepad2.a){
                liftTest(1200);
            }
            if(gamepad2.b){
                liftTest(500);
            }
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
            if (gamepad1.x) {//
                SwervePod1.setPosition(0.8);
                SwervePod2.setPosition(0.2);
                SwervePod3.setPosition(0.8);
                SwervePod4.setPosition(0.2);
                currentLatchIndex = 1 % Latch.length;
            }
            if (gamepad1.y) {//
                SwervePod1.setPosition(0.68);
                SwervePod2.setPosition(0.35);
                SwervePod3.setPosition(0.68);
                SwervePod4.setPosition(0.35);
                currentLatchIndex = 0 % Latch.length;
            }
            if (gamepad1.b) {//
                SwervePod1.setPosition(0.8);
                SwervePod2.setPosition(0.2);
                SwervePod3.setPosition(0.8);
                SwervePod4.setPosition(0.2);
                currentLatchIndex = 1 % Latch.length;
            }
            if (gamepad1.right_bumper) {//
                SwervePod1.setPosition(0.35);
                SwervePod2.setPosition(0.35);
                SwervePod3.setPosition(0.35);
                SwervePod4.setPosition(0.35);
                currentLatchIndex = 0 % Latch.length;
            }
            if (gamepad1.left_bumper) {//checked
                SwervePod1.setPosition(0.685);
                SwervePod2.setPosition(0.68);
                SwervePod3.setPosition(0.68);
                SwervePod4.setPosition(0.685);
                currentLatchIndex = 0 % Latch.length;
            }

            RobotLog.i("RF9958  –Runtime: " + runtime.seconds()
                    + "  RF9958 - lift encoder " + IntakeLift.getCurrentPosition());
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
    public void liftTest(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        IntakeLift.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( IntakeLift.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( IntakeLift.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                IntakeLift.setPower(0.25);
            }
            //If statement that checks if the motors current position is less then the target
            else if( IntakeLift.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                IntakeLift.setPower(0.25);
            }
            sleep(250l);
            IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
//    public void liftTestRight(int target){
//        //Sets the new target position for the glyph lifter
//        // RightLiftMotor.setTargetPosition(target);
//        LiftMotorRight.setTargetPosition(target);
//        //Turns on RUN_TO_POSITION
//        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LiftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //If statement that ask if the motor is busy
//        if(LiftMotorRight.isBusy()){
//            //If statement that checks if the motors current position is more then the target
//            if(LiftMotorRight.getCurrentPosition() > target){
//                //If the current position is more than the target, set motor power to 40%
//                // RightLiftMotor.setPower(0.45);
//                LiftMotorRight.setPower(0.85);
//            }
//            //If statement that checks if the motors current position is less then the target
//            else if(LiftMotorRight.getCurrentPosition() < target){
//                //If the current position is more than the target, set motor power to 60%
//                // RightLiftMotor.setPower(0.5);
//                LiftMotorRight.setPower(1.0);
//            }
//        }
//    }
}