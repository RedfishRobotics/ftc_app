package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto library Test", group="Linear Opmode")

public class Auto_test_with_library extends LinearOpMode {

    Auto_Library autoLibrary = new Auto_Library();

    @Override
    public void runOpMode(){
        autoLibrary.init(hardwareMap);
        waitForStart();
        encoderStrafeLeft(0.75, 5, 5, 5);
        sleep(1000);
        encoderStrafeRight(0.75, 5, 5, 5);
        sleep(1000);
        autoLibrary.runIntakeForward();
        sleep(1000);
        autoLibrary.stopIntake();
        sleep(1000);
        autoLibrary.flipperFlatwithStop();
        sleep(1000);
        autoLibrary.flipperDump();
        sleep(1000);
        autoLibrary.flipperDown();
        sleep(1000);
        encoderDrive(0.75, 5, 5, 5);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;//init the variable newLeftTarget
        int newRightTarget;//init the variable newRightTarget

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = autoLibrary.leftFrontDrive.getCurrentPosition() + (int)(leftInches * autoLibrary.COUNTS_PER_INCH);
            newRightTarget = autoLibrary.rightFrontDrive.getCurrentPosition() + (int)(rightInches * autoLibrary.COUNTS_PER_INCH);
            autoLibrary.leftFrontDrive.setTargetPosition(newLeftTarget);
            autoLibrary.leftRearDrive.setTargetPosition(newLeftTarget);
            autoLibrary.rightFrontDrive.setTargetPosition(newRightTarget);
            autoLibrary.rightRearDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            autoLibrary.runtime.reset();
            autoLibrary.leftFrontDrive.setPower(speed);
            autoLibrary.leftRearDrive.setPower(speed);
            autoLibrary.rightFrontDrive.setPower(speed);
            autoLibrary.rightRearDrive.setPower(speed);

            //While the motors and OpMode is running, return telemetry on the motors
            while (opModeIsActive() &&
                    (autoLibrary.runtime.seconds() < timeoutS) &&
                    (autoLibrary.leftFrontDrive.isBusy() && autoLibrary.rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        autoLibrary.leftFrontDrive.getCurrentPosition(),
                        autoLibrary.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            autoLibrary.leftFrontDrive.setPower(0);
            autoLibrary.leftRearDrive.setPower(0);
            autoLibrary.rightFrontDrive.setPower(0);
            autoLibrary.rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void encoderStrafeLeft(double speed,
                                  double leftInches, double rightInches,
                                  double timeoutS) {
        int newLeftTarget;//init the variable newLeftTarget
        int newRightTarget;//init the variable newRightTarget

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = autoLibrary.leftFrontDrive.getCurrentPosition() + (int)(leftInches * autoLibrary.COUNTS_PER_INCH);
            newRightTarget = autoLibrary.rightFrontDrive.getCurrentPosition() + (int)(rightInches * autoLibrary.COUNTS_PER_INCH);
            autoLibrary.leftFrontDrive.setTargetPosition(newLeftTarget);
            autoLibrary.leftRearDrive.setTargetPosition(-newLeftTarget);
            autoLibrary.rightFrontDrive.setTargetPosition(-newRightTarget);
            autoLibrary.rightRearDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            autoLibrary.runtime.reset();
            autoLibrary.leftFrontDrive.setPower(speed);
            autoLibrary.leftRearDrive.setPower(speed);
            autoLibrary.rightFrontDrive.setPower(speed);
            autoLibrary.rightRearDrive.setPower(speed);

            //While the motors and OpMode is running, return telemetry on the motors
            while (opModeIsActive() &&
                    (autoLibrary.runtime.seconds() < timeoutS) &&
                    (autoLibrary.leftFrontDrive.isBusy() && autoLibrary.rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        autoLibrary.leftFrontDrive.getCurrentPosition(),
                        autoLibrary.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            autoLibrary.leftFrontDrive.setPower(0);
            autoLibrary.leftRearDrive.setPower(0);
            autoLibrary.rightFrontDrive.setPower(0);
            autoLibrary.rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void encoderStrafeRight(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS) {
        int newLeftTarget;//init the variable newLeftTarget
        int newRightTarget;//init the variable newRightTarget

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = autoLibrary.leftFrontDrive.getCurrentPosition() + (int)(leftInches * autoLibrary.COUNTS_PER_INCH);
            newRightTarget = autoLibrary.rightFrontDrive.getCurrentPosition() + (int)(rightInches * autoLibrary.COUNTS_PER_INCH);
            autoLibrary.leftFrontDrive.setTargetPosition(-newLeftTarget);
            autoLibrary.leftRearDrive.setTargetPosition(newLeftTarget);
            autoLibrary.rightFrontDrive.setTargetPosition(newRightTarget);
            autoLibrary.rightRearDrive.setTargetPosition(-newRightTarget);

            // Turn On RUN_TO_POSITION
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            autoLibrary.runtime.reset();
            autoLibrary.leftFrontDrive.setPower(speed);
            autoLibrary.leftRearDrive.setPower(speed);
            autoLibrary.rightFrontDrive.setPower(speed);
            autoLibrary.rightRearDrive.setPower(speed);


            //While the motors and OpMode is running, return telemetry on the motors
            while (opModeIsActive() &&
                    (autoLibrary.runtime.seconds() < timeoutS) &&
                    (autoLibrary.leftFrontDrive.isBusy() && autoLibrary.rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        autoLibrary.leftFrontDrive.getCurrentPosition(),
                        autoLibrary.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            autoLibrary.leftFrontDrive.setPower(0);
            autoLibrary.leftRearDrive.setPower(0);
            autoLibrary.rightFrontDrive.setPower(0);
            autoLibrary.rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
