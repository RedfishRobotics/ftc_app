package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOp_Library  extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor RightLiftMotor = null;
    private DcMotor LeftLiftMotor = null;
    private DcMotor RightIntakeMotor = null;
    private DcMotor LeftIntakeMotor = null;
    private Servo rightFlip = null;
    private Servo leftFlip = null;
    private CRServo PITA_1 = null;
    private CRServo PITA_2 = null;
    private CRServo relicReel = null;
    private Servo ColorArmTurn = null;
    private Servo ColorSensorArm = null;
    private Servo glyphStop = null;
    private Servo relicPivot = null;
    private Servo relicClaw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // set references for config
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_FM");
        leftRearDrive = hardwareMap.get(DcMotor.class, "Left_RM");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_FM");
        rightRearDrive = hardwareMap.get(DcMotor.class, "Right_RM");
        RightLiftMotor = hardwareMap.get(DcMotor.class, "RightLiftMotor");
        LeftLiftMotor = hardwareMap.get(DcMotor.class, "LeftLiftMotor");
        RightIntakeMotor = hardwareMap.get(DcMotor.class, "RightIntakeMotor");
        LeftIntakeMotor = hardwareMap.get(DcMotor.class, "LeftIntakeMotor");
        rightFlip = hardwareMap.get(Servo.class, "Right_Flip");
        leftFlip = hardwareMap.get(Servo.class, "Left_Flip");
        PITA_1 = hardwareMap.get(CRServo.class, "Intake_Stop_Right");
        PITA_2 = hardwareMap.get(CRServo.class, "Intake_Stop_Left");
        ColorArmTurn = hardwareMap.get(Servo.class, "colorArmTurn");
        ColorSensorArm = hardwareMap.get(Servo.class, "colorSensorArm");
        glyphStop = hardwareMap.get(Servo.class, "Glyph_Stop");
        relicReel = hardwareMap.get(CRServo.class, "Relic_Reel");
        relicPivot = hardwareMap.get(Servo.class, "Relic_Pivot");
        relicClaw = hardwareMap.get(Servo.class, "Relic_Claw");

        // Set the rotation servo for extended PWM range
        // if (relicPivot.getController() instanceof ServoControllerEx) {
        // // Confirm its an extended range servo controller before we try to set to avoid crash
        // ServoControllerEx theControl = (ServoControllerEx) relicPivot.getController();
        // int thePort = relicPivot.getPortNumber();
        // PwmControl.PwmRange theRange = new PwmControl.PwmRange(553, 2500);
        // theControl.setServoPwmRange(thePort, theRange);
        // }

        // set the direction of the motor
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        RightLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        RightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        PITA_1.setDirection(CRServo.Direction.REVERSE);
        PITA_2.setDirection(CRServo.Direction.FORWARD);
        relicReel.setDirection(CRServo.Direction.FORWARD);

        RightIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rightFlipPosition = 1.0;
        // double IntakeStopPosition = 0.8;
        double leftFlipPosition = 0.0;
        double colorArmTurnPosition = 0.88;
        double colorSensorPosition = 0.77;
        double glyphStopPosition = 0.3;
        //double relicPivotPosition = 0.05;

        ColorArmTurn.setPosition(colorArmTurnPosition);
        ColorSensorArm.setPosition(colorSensorPosition);
        rightFlip.setPosition(rightFlipPosition);
        leftFlip.setPosition(leftFlipPosition);
        glyphStop.setPosition(glyphStopPosition);
        //relicPivot.setPosition(relicPivotPosition);

        // Wait for the game to start (driver presses PL
    }
    public void runIntake() {
        if (gamepad1.right_bumper) {
            RightIntakeMotor.setPower(0.8);
            LeftIntakeMotor.setPower(0.8);
        }
        if (gamepad1.left_bumper) {
            RightIntakeMotor.setPower(0);
            LeftIntakeMotor.setPower(0);
        }
        if (gamepad1.dpad_down) {
            RightIntakeMotor.setPower(0.6);
            LeftIntakeMotor.setPower(0.6);
        }
        if (gamepad1.dpad_up) {
            RightIntakeMotor.setPower(-0.55);
            LeftIntakeMotor.setPower(-0.55);
        }
    }
    public void relicClaw() {
        if (gamepad2.left_bumper) {//closed
            relicClaw.setPosition(1.0);
        }
        if (gamepad2.right_bumper) {//open
            relicClaw.setPosition(0.3);

        }
    }
    public void pivotRelic() {
        if (gamepad2.dpad_down) {//down
            relicPivot.setPosition(0.225);
        }
        if (gamepad2.dpad_up) {//up
            relicPivot.setPosition(0.75);
        }
    }
    public void deployRelic() {
        if (gamepad2.y) {//deployment
            relic_Deployment();
        }
    }
    public void runRelicReel() {
        if (-gamepad2.left_stick_y > 0.4 || -gamepad2.left_stick_y < -0.4) {
            relicReel.setPower(gamepad2.left_stick_y);
        } else if (-gamepad2.right_stick_y > 0.4 || -gamepad2.right_stick_y < -0.4) {
            relicReel.setPower(gamepad2.right_stick_y / 4);
        } else {
            relicReel.setPower(0);
        }
    }
    // if(gamepad1.dpad_left){//back
    //     relicReel.setPower(0.6);
    // }
    // if(gamepad1.dpad_right){//out
    //     relicReel.setPower(-0.8);
    // }
    // if(gamepad1.y){//stopped
    //     relicReel.setPower(0);
    // }
    public void flipperFlatwithStop() {
        if (gamepad2.b) {//flat
            glyphStop.setPosition(1.0);
            sleep(350);
            glyphStop.setPosition(0.9);
            sleep(200);
            leftFlip.setPosition(0.15);
            rightFlip.setPosition(0.85);
        }
    }
    public void lifterDown() {
        if (gamepad2.a) {// lifter down
            rightLift(10);
            leftLift(10);
        }
    }
    public void lifterUp() {
        if (gamepad2.x && leftFlip.getPosition() >= 0.15) {//lifter up
            glyphStop.setPosition(0.85);
            sleep(350);
            rightLift(3500);
            leftLift(3500);
        }
    }
    public void flipperDump() {
        if (gamepad1.b && leftFlip.getPosition() >= 0.15) {
            glyphStop.setPosition(0.85);
            sleep(250);
            leftFlip.setPosition(0.7);
            rightFlip.setPosition(0.3);
            // ColorArmTurn.setPosition(0.88);
        }
    }
    // if(gamepad1.right_bumper){
    // //   jewelKnockRight(250, 0.34);
    // }
    public void flipperFlat() {
        if (gamepad1.x) {
            leftFlip.setPosition(0.15);
            rightFlip.setPosition(0.85);
        }
    }
    public void flipperDown() {
        if (gamepad1.a && LeftLiftMotor.getCurrentPosition() < 20) {
            leftFlip.setPosition(0.0);
            rightFlip.setPosition(1.0);
            sleep(350);
            glyphStop.setPosition(0.3);
        }
    }
    public void driveAndStrafe() {
        if (gamepad1.left_trigger > 0.2) {
            // set the motors to strafe to the left
            leftFrontDrive.setPower(gamepad1.left_trigger);
            leftRearDrive.setPower(-gamepad1.left_trigger);
            rightFrontDrive.setPower(-gamepad1.left_trigger);
            rightRearDrive.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.2) {
            // // set the motors to strafe to the right
            leftFrontDrive.setPower(-gamepad1.right_trigger);
            leftRearDrive.setPower(gamepad1.right_trigger);
            rightFrontDrive.setPower(gamepad1.right_trigger);
            rightRearDrive.setPower(-gamepad1.right_trigger);
        } else {
            rightFrontDrive.setPower(-gamepad1.right_stick_y);
            rightRearDrive.setPower(-gamepad1.right_stick_y);
            leftFrontDrive.setPower(-gamepad1.left_stick_y);
            leftRearDrive.setPower(-gamepad1.left_stick_y);
        }
    }

    //   jewelKnockLeft(250, 0.78);
    // }if(gamepad1.y){
    // //   ColorSensorArm.setPosition(0.76);
    // }if(gamepad1.dpad_down){
    // //   ColorSensorArm.setPosition(0.2);
    // }
    public void telemetry(){
        telemetry.addLine("Right Lift Motor: " + RightLiftMotor.getCurrentPosition());
        telemetry.addLine("Left Lift Motor: " + LeftLiftMotor.getCurrentPosition());
        telemetry.update();
    }
    public void rightLift(int target){
        //Sets the new target position for the glyph lifter
        RightLiftMotor.setTargetPosition(target);
        // LeftLiftMotor.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if(RightLiftMotor.isBusy() /*&& LeftLiftMotor.isBusy()*/){
            //If statement that checks if the motors current position is more then the target
            if(RightLiftMotor.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                RightLiftMotor.setPower(0.85);
                // LeftLiftMotor.setPower(0.45);
            }
            //If statement that checks if the motors current position is less then the target
            else if(RightLiftMotor.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                RightLiftMotor.setPower(1.0);
                // LeftLiftMotor.setPower(0.5);
            }
        }
    }
    public void leftLift(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        LeftLiftMotor.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if(LeftLiftMotor.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if(LeftLiftMotor.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                LeftLiftMotor.setPower(0.85);
            }
            //If statement that checks if the motors current position is less then the target
            else if(LeftLiftMotor.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                LeftLiftMotor.setPower(1.0);
            }
        }
    }
    void jewelKnockRight(int sleepPerTurn, double ArmTurnValue){
        double colorArmTurnPosition = 0.62;
        while(ColorArmTurn.getPosition() > ArmTurnValue){
            colorArmTurnPosition -= 0.02;
            ColorArmTurn.setPosition(colorArmTurnPosition);
            sleep(sleepPerTurn);
        }
    }
    void jewelKnockLeft(int sleepPerTurn, double ArmTurnValue){
        double colorArmTurnPosition = 0.62;
        while(ColorArmTurn.getPosition() < ArmTurnValue){
            colorArmTurnPosition += 0.02;
            ColorArmTurn.setPosition(colorArmTurnPosition);
            sleep(sleepPerTurn);
        }
    }
    public void relic_Deployment(){
        relicReel.setPower(0);
        //Sets the hook to the semi-opened position
        relicClaw.setPosition(0.95);
        //Sets the relic flip to till back for deployment
        relicPivot.setPosition(0.17);
        //Sleeps to let the robot settle
        sleep(1500);
        //While loop that checks the hook position for an increment
        relicClaw.setPosition(0.5);
        sleep(250);
        relicReel.setPower(0.6);
        sleep(250);
        relicReel.setPower(0);
    }

}
