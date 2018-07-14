package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

@Autonomous(name="Auto library Test", group="Linear Opmode")

public class Auto_test_with_library extends LinearOpMode {

    Auto_Library autoLibrary = new Auto_Library();

    @Override
    public void runOpMode(){
        autoLibrary.init(hardwareMap);
        composeTelemetry();
        telemetry.update();
        waitForStart();
        gyroTurn(0.5, 270, 0.011);
        sleep(1000);
//        encoderStrafeLeft(0.75, 5, 5, 5);
//        sleep(1000);
//        encoderStrafeRight(0.75, 5, 5, 5);
//        sleep(1000);
//        autoLibrary.runIntakeForward();
//        sleep(1000);
//        autoLibrary.stopIntake();
//        sleep(1000);
//        autoLibrary.flipperFlatwithStop();
//        sleep(1000);
//        autoLibrary.flipperDump();
//        sleep(1000);
//        autoLibrary.flipperDown();
//        sleep(1000);
//        encoderDrive(0.75, 5, 5, 5);
    }
    public void gyroTurn (  double speed, double angle, double coefficient) {

        telemetry.addLine("DM10337- gyroTurn start  speed:" + speed +
                "  heading:" + angle);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, coefficient)) {
            // Allow time for other processes to run.
            // onHeading() does the work of turning us
            sleep(1);;
        }

        telemetry.addLine("DM10337- gyroTurn done   heading actual:" + readGyro());
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        int HEADING_THRESHOLD = 5;
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            // Close enough so no need to move
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            // Calculate motor powers
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        autoLibrary.leftFrontDrive.setPower(leftSpeed);
        autoLibrary.rightFrontDrive.setPower(rightSpeed);
        autoLibrary.leftRearDrive.setPower(leftSpeed);
        autoLibrary.rightRearDrive.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        autoLibrary.angles   = autoLibrary.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - autoLibrary.angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     * Record the current heading and use that as the 0 heading point for gyro reads
     * @return
     */
    void zeroGyro() {
        double headingBias;
        autoLibrary.angles = autoLibrary.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingBias = autoLibrary.angles.firstAngle;
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            autoLibrary.angles   = autoLibrary.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            autoLibrary.gravity  = autoLibrary.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return autoLibrary.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return autoLibrary.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(autoLibrary.angles.angleUnit, autoLibrary.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(autoLibrary.angles.angleUnit, autoLibrary.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(autoLibrary.angles.angleUnit, autoLibrary.angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return autoLibrary.gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(autoLibrary.gravity.xAccel*autoLibrary.gravity.xAccel
                                        + autoLibrary.gravity.yAccel*autoLibrary.gravity.yAccel
                                        + autoLibrary.gravity.zAccel*autoLibrary.gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    // /**
    //  * Read the current heading direction.  Use a heading bias if we recorded one at start to account for drift during
    //  * the init phase of match
    //  *
    //  * @return      Current heading (Z axis)
    //  */
    double readGyro() {
        double headingBias = autoLibrary.angles.firstAngle;
        autoLibrary.angles = autoLibrary.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return autoLibrary.angles.firstAngle - headingBias;
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
