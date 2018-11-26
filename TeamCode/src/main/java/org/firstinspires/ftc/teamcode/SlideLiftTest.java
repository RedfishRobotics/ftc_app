package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="IntakeLiftTest", group="Pushbot")
public class SlideLiftTest extends LinearOpMode {

    public DcMotor liftSlideRight = null;
    public DcMotor liftSlideLeft = null;
    public DigitalChannel magneticSwitch = null;
    public CRServo flipperServo = null;

    @Override
    public void runOpMode(){

        liftSlideLeft = hardwareMap.get(DcMotor.class, "Lift_Slide_Left");
        liftSlideRight = hardwareMap.get(DcMotor.class, "Lift_Slide_Right");
        magneticSwitch = hardwareMap.get(DigitalChannel.class, "Hull_Effect");
        flipperServo = hardwareMap.get(CRServo.class, "Flip_Servo");

        liftSlideRight.setDirection(DcMotor.Direction.REVERSE);
        liftSlideLeft.setDirection(DcMotor.Direction.REVERSE);

        liftSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()){
//            if (magneticSwitch.getState() == true) {
//                telemetry.addData("Magnetic Switch", "Is Opened");
//                flipperServo.setPower(0.5);
//            } else {
//                telemetry.addData("Magnetic Switch", "Is Closed");
//                flipperServo.setPower(0);
//            }

            telemetry.update();

            if(gamepad1.y){

            }
            if(gamepad1.x){
                LiftTest(0);
            }
            if(gamepad1.b){
                LiftTest(2200);
                sleep(1000);
                while(magneticSwitch.getState() == true){
                    flipperServo.setPower(0.75);
                }
                flipperServo.setPower(0);
            }
//            liftSlideLeft.setPower(-gamepad1.right_stick_y);
//            liftSlideRight.setPower(-gamepad1.right_stick_y);
        }
    }
    public void LiftTest(int target){
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        liftSlideLeft.setTargetPosition(target);
        liftSlideRight.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if( liftSlideLeft.isBusy()){
            //If statement that checks if the motors current position is more then the target
            if( liftSlideLeft.getCurrentPosition() > target){
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                liftSlideLeft.setPower(0.25);
                liftSlideRight.setPower(0.25);
            }
            //If statement that checks if the motors current position is less then the target
            else if( liftSlideLeft.getCurrentPosition() < target){
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                liftSlideLeft.setPower(0.75);
                liftSlideRight.setPower(0.75);
            }
        }
    }
}
