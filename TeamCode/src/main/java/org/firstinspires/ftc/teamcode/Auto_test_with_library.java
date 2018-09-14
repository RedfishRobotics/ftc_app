package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.util.Locale;

@Autonomous(name="Auto library Test", group="Linear Opmode")

public class Auto_test_with_library extends LinearOpMode {

    VuforiaLocalizer vuforia;
    Auto_Library autoLibrary = new Auto_Library();


    public MovingAvg gyroErrorAvg = new MovingAvg(30);
    static final double P_DRIVE_COEFF_1 = 0.01;  // Larger is more responsive, but also less accurate
    static final double P_DRIVE_COEFF_2 = 0.25;  // Intenionally large so robot "wiggles" around the target setpoint while driving

    @Override
    public void runOpMode() throws InterruptedException {
        autoLibrary.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVuforia = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parametersVuforia.vuforiaLicenseKey = "AeyEUJr/////AAABmeFmvlzTdkEAiH3nHjERdd+Llh9YjOwGt7MJzlc6lwgtipxMyv3XcDmgJ9xt4hP+jxTG0U9/ryXj5p9dCnKDxdKUk0eXb7+916/0BpGO5Oo3sIu/wj56lSatbA6e/vHUHtawRO3XodseNo8YN3yQLPlEYDh6NuRP+m3559sMhYaJJnFdnieUEtgHV/Bjiv1P3wNy5dGDX541b+fBOiXX1xIq+Bt/bZ/c8dRZweH/56c8pwxszEZ3dLBr9e6IMqZ1q31B4dE1az8QzF3vHzmDHLwVu1Nw5noOeN3g7QEbgseLuUISxl8EvSHzcwumkAszmMaO+W0d10dMbgeuQnZgjInLfI/qhkVn22jewMn3whu3";
        parametersVuforia.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersVuforia);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        composeTelemetry();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            relicTrackables.activate();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

//            if (autoLibrary.magneticSwitch.getState() == true) {
//                telemetry.addData("Magnetic Switch", "Is Opened");
//            } else {
//                telemetry.addData("Magnetic Switch", "Is Closed");
//            }

//            telemetry.update();
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderDrive(0.8, 24.0, 5.0, false, 10.0, false, false, 1.0);
//                if (vuMark.equals(RelicRecoveryVuMark.CENTER)){
//                    telemetry.addLine("Center");
//                    telemetry.update();
//                }
//                else if (vuMark.equals(RelicRecoveryVuMark.LEFT)){
//                    telemetry.addLine("Left");
//                    telemetry.update();
//                }
//                else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)){
//                    telemetry.addLine("Right");
//                    telemetry.update();
//                }
//                else {
//                    telemetry.addLine("Default");
//                    telemetry.update();
//                }
//            gyroTurn(0.5, 270, 0.011);
//            sleep(1000);
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
            stop();
        }
    }
    public void encoderDrive(double speed,
                             double distance,
                             double timeout,
                             boolean useGyro,
                             double heading,
                             boolean aggressive,
                             boolean userange,
                             double maintainRange) throws InterruptedException {

        // Calculated encoder targets
        int newLFTarget;
        int newRFTarget;
        int newLRTarget;
        int newRRTarget;

        // The potentially adjusted current target heading
        double curHeading = heading;

        // Speed ramp on start of move to avoid wheel slip
        final double MINSPEED = 0.30;           // Start at this power
        final double SPEEDINCR = 0.015;         // And increment by this much each cycle
        double curSpeed;                        // Keep track of speed as we ramp

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            RobotLog.i("DM10337- Starting encoderDrive speed:" + speed +
                    "  distance:" + distance + "  timeout:" + timeout +
                    "  useGyro:" + useGyro + " heading:" + heading + "  maintainRange: " + maintainRange);

            // Calculate "adjusted" distance  for each side to account for requested turn during run
            // Purpose of code is to have PIDs closer to finishing even on curved moves
            // This prevents jerk to one side at stop
            double leftDistance = distance;
            double rightDistance = distance;
            if (useGyro) {
                // We are gyro steering -- are we requesting a turn while driving?
                double headingChange = getError(curHeading) * Math.signum(distance);
                if (Math.abs(headingChange) > 5.0) {
                    //Heading change is significant enough to account for
                    if (headingChange > 0.0) {
                        // Assume 15.25 inch wheelbase
                        // Add extra distance to the wheel on outside of turn
                        rightDistance += Math.signum(distance) * 2 * 3.1415 * 12 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted R distance:" + rightDistance);
                    } else {
                        // Assume 15.25 inch wheelbase
                        // Add extra distance from the wheel on inside of turn
                        // headingChange is - so this is increasing the left distance
                        leftDistance -= Math.signum(distance) * 2 * 3.1415 * 12 * headingChange / 360.0;
                        RobotLog.i("DM10337 -- Turn adjusted L distance:" + leftDistance);
                    }
                }
            }

            // Determine new target encoder positions, and pass to motor controller
            newLFTarget = autoLibrary.leftFrontDrive.getCurrentPosition() + (int)(leftDistance * autoLibrary.COUNTS_PER_INCH);
            newLRTarget = autoLibrary.leftRearDrive.getCurrentPosition() + (int)(leftDistance * autoLibrary.COUNTS_PER_INCH);
            newRFTarget = autoLibrary.rightFrontDrive.getCurrentPosition() + (int)(rightDistance * autoLibrary.COUNTS_PER_INCH);
            newRRTarget = autoLibrary.rightRearDrive.getCurrentPosition() + (int)(rightDistance * autoLibrary.COUNTS_PER_INCH);

            while(autoLibrary.leftFrontDrive.getTargetPosition() != newLFTarget){
                autoLibrary.leftFrontDrive.setTargetPosition(newLFTarget);
                sleep(1);
            }
            while(autoLibrary.rightFrontDrive.getTargetPosition() != newRFTarget){
                autoLibrary.rightFrontDrive.setTargetPosition(newRFTarget);
                sleep(1);
            }
            while(autoLibrary.leftRearDrive.getTargetPosition() != newLRTarget){
                autoLibrary.leftRearDrive.setTargetPosition(newLRTarget);
                sleep(1);
            }
            while(autoLibrary.rightRearDrive.getTargetPosition() != newRRTarget){
                autoLibrary.rightRearDrive.setTargetPosition(newRRTarget);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            autoLibrary.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoLibrary.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            autoLibrary.runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED,speed);

            // Set the motors to the starting power
            autoLibrary.leftFrontDrive.setPower(Math.abs(curSpeed));
            autoLibrary.rightFrontDrive.setPower(Math.abs(curSpeed));
            autoLibrary.leftRearDrive.setPower(Math.abs(curSpeed));
            autoLibrary.rightRearDrive.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (autoLibrary.runtime.seconds() < timeout) &&
                    autoLibrary.leftFrontDrive.isBusy() &&
                    autoLibrary.leftRearDrive.isBusy() &&
                    autoLibrary.rightFrontDrive.isBusy() &&
                    autoLibrary.rightRearDrive.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro){

                    // adjust relative speed based on heading
                    double error = getError(curHeading);

                    updateGyroErrorAvg(error);

                    double steer = getSteer(error,
                            (aggressive?P_DRIVE_COEFF_2:P_DRIVE_COEFF_1));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                autoLibrary.leftFrontDrive.setPower(Math.abs(leftSpeed));
                autoLibrary.rightFrontDrive.setPower(Math.abs(rightSpeed));
                autoLibrary.leftRearDrive.setPower(Math.abs(leftSpeed));
                autoLibrary.rightRearDrive.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                sleep(1);;
            }


            RobotLog.i("DM10337- encoderDrive done" +
                    "  lftarget: " +newLFTarget + "  lfactual:" + autoLibrary.leftFrontDrive.getCurrentPosition() +
                    "  lrtarget: " +newLFTarget + "  lractual:" + autoLibrary.leftRearDrive.getCurrentPosition() +
                    "  rftarget: " +newRFTarget + "  rfactual:" + autoLibrary.rightFrontDrive.getCurrentPosition() +
                    "  rrtarget: " +newRFTarget + "  rractual:" + autoLibrary.rightRearDrive.getCurrentPosition() +
                    "  heading:" + readGyro());

            RobotLog.i ("DM10337 - Gyro error average: " + gyroErrorAvg.average());

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

    public void updateGyroErrorAvg(double error) {
        gyroErrorAvg.add(Math.abs(error));
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
    /*public void encoderDrive(double speed,
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
    }*/
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
