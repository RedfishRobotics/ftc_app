package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Latching TeleOp Test", group="Linear Opmode")

public class Latching_Teleop extends LinearOpMode {

    TeleOp_Library teleOpLibrary = new TeleOp_Library();

    private static final double[] Latch = new double[]{0, 1};
    private int currentLatchIndex;
    private boolean yPressedLast;

    @Override
    public void runOpMode() {
        teleOpLibrary.init(hardwareMap);
        this.currentLatchIndex = 0;
        this.yPressedLast = false;

        waitForStart();
        while (opModeIsActive()) {
            final boolean yPressed = this.gamepad1.y;
            if (yPressed && !this.yPressedLast)
                this.currentLatchIndex = (this.currentLatchIndex + 1) % Latch.length;
            this.yPressedLast = yPressed;
            if(currentLatchIndex == 0){
                if (gamepad1.left_trigger > 0.2) {
                    teleOpLibrary.strafeLeft(gamepad1.left_trigger);
                } if (gamepad1.right_trigger > 0.2) {
                    teleOpLibrary.strafeRight(gamepad1.right_trigger);
                } else {
                    teleOpLibrary.driveForward(gamepad1.left_stick_y, gamepad1.right_stick_y);
                }
            }
            else if(currentLatchIndex == 1){
                double drive;
                double turn;
                drive = -gamepad1.left_stick_y;
                turn  =  gamepad1.right_stick_x;
                teleOpLibrary.rightFrontDrive.setPower(drive - turn);
                teleOpLibrary.rightRearDrive.setPower(drive + turn);
                teleOpLibrary.leftFrontDrive.setPower(drive + turn);
                teleOpLibrary.leftRearDrive.setPower(drive - turn);
            }
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
            if (-gamepad2.left_stick_y > 0.4 || -gamepad2.left_stick_y < -0.4) {
                teleOpLibrary.relicReel.setPower(gamepad2.left_stick_y);
            } else if (-gamepad2.right_stick_y > 0.4 || -gamepad2.right_stick_y < -0.4) {
                teleOpLibrary.relicReel.setPower(gamepad2.right_stick_y / 4);
            } else {
                teleOpLibrary.relicReel.setPower(0);
            }
            telemetry.addLine("Right Lift Motor: " + teleOpLibrary.RightLiftMotor.getCurrentPosition());
            telemetry.addLine("Left Lift Motor: " + teleOpLibrary.LeftLiftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
