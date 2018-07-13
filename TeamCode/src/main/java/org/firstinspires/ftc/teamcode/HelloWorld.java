package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Library Test", group="Linear Opmode")

public class HelloWorld extends LinearOpMode {

    Auto_Library autoLibrary = new Auto_Library();
    TeleOp_Library teleOpLibrary = new TeleOp_Library();


    @Override
    public void runOpMode() {
        //autoLibrary.init(hardwareMap);
        teleOpLibrary.init(hardwareMap);
        waitForStart();
//            autoLibrary.encoderDrive(0.5, 25, 25, 5);
//            autoLibrary.rightFrontDrive.setPower(0.5);
//            sleep(1000);
//            autoLibrary.rightFrontDrive.setPower(0.0);
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                teleOpLibrary.runIntakeForward();
            }
            if (gamepad1.left_bumper) {
                teleOpLibrary.stopIntake();
            }
            if (gamepad1.dpad_down) {
                teleOpLibrary.runIntakeHalf();
            }
            if (gamepad1.dpad_up) {
                teleOpLibrary.runIntakeBackward();
            }
            if (gamepad2.left_bumper) {
                teleOpLibrary.relicClawClose();
            }
            if (gamepad2.right_bumper) {
                teleOpLibrary.relicClawOpen();
            }
            if (gamepad2.dpad_down) {
                teleOpLibrary.pivotRelicDown();
            }
            if (gamepad2.dpad_up) {
                teleOpLibrary.pivotRelicUp();
            }
            if (gamepad2.y) {
                teleOpLibrary.deployRelic();
            }
            if (gamepad2.b) {
                teleOpLibrary.flipperFlatwithStop();
            }
            if (gamepad1.x) {
                teleOpLibrary.flipperFlat();
            }
            if (gamepad1.b && teleOpLibrary.leftFlip.getPosition() >= 0.15) {
                teleOpLibrary.flipperDump();
            }
            if (gamepad1.a && teleOpLibrary.LeftLiftMotor.getCurrentPosition() < 20) {
                teleOpLibrary.flipperDown();
            }
            if (gamepad2.a) {
                teleOpLibrary.lifterDown();
            }
            if (gamepad2.x && teleOpLibrary.leftFlip.getPosition() >= 0.15) {
                teleOpLibrary.lifterUp();
            }
            if (gamepad1.left_trigger > 0.2) {
                teleOpLibrary.strafeLeft(gamepad1.left_trigger);
            } if (gamepad1.right_trigger > 0.2) {
                teleOpLibrary.strafeRight(gamepad1.right_trigger);
            } else {
               teleOpLibrary.driveForward(gamepad1.left_stick_y, gamepad1.right_stick_y);
            }
//            //teleOpLibrary.telemetry();
//            //This is a test of Git
//            //Another test
        }
    }
}
