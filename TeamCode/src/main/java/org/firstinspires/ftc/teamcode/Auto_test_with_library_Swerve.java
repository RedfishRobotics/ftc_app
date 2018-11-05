package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="Auto library Test Swerve", group="Linear Opmode")

public class Auto_test_with_library_Swerve extends LinearOpMode {
    Auto_Library autoLibrary = new Auto_Library();
    Auto_Library_Swerve autoSwerve = new Auto_Library_Swerve();


    public MovingAvg gyroErrorAvg = new MovingAvg(30);
    static final double P_DRIVE_COEFF_1 = 0.01;  // Larger is more responsive, but also less accurate
    static final double P_DRIVE_COEFF_2 = 0.25;  // Intenionally large so robot "wiggles" around the target setpoint while driving
    int goldMineralPosition = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // DogeCV detector
    GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        autoSwerve.init(hardwareMap);
        autoLibrary.init(hardwareMap);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key
        parameters.vuforiaLicenseKey = "AeyEUJr/////AAABmeFmvlzTdkEAiH3nHjERdd+Llh9YjOwGt7MJzlc6lwgtipxMyv3XcDmgJ9xt4hP+jxTG0U9/ryXj5p9dCnKDxdKUk0eXb7+916/0BpGO5Oo3sIu/wj56lSatbA6e/vHUHtawRO3XodseNo8YN3yQLPlEYDh6NuRP+m3559sMhYaJJnFdnieUEtgHV/Bjiv1P3wNy5dGDX541b+fBOiXX1xIq+Bt/bZ/c8dRZweH/56c8pwxszEZ3dLBr9e6IMqZ1q31B4dE1az8QzF3vHzmDHLwVu1Nw5noOeN3g7QEbgseLuUISxl8EvSHzcwumkAszmMaO+W0d10dMbgeuQnZgjInLfI/qhkVn22jewMn3whu3";

        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        //Setup trackables
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT  = 215;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 324;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 165;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate the targets
        targetsRoverRuckus.activate();

        // Initialize the detector
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

        // Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
        composeTelemetry();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            webcamScan();
            encoderDrive(0.4, 4,2, false, 2, true, false, 0);
            sleep(500);
            if(goldMineralPosition == 1){
                autoSwerve.spinPosition();
                sleep(250);
                gyroTurn(0.4, 45, 0.011);
                sleep(250);
                autoSwerve.straightPosition();
                sleep(250);
                encoderDrive(0.4, 22,4, false, 2, true, false, 0);
                //sleep(250);
                //autoSwerve.leftKnock();
            } else if(goldMineralPosition == 2){
                autoSwerve.SwervePod1.setPosition(0.6);
                autoSwerve.SwervePod2.setPosition(0.6);
                autoSwerve.SwervePod3.setPosition(0.6);
                autoSwerve.SwervePod4.setPosition(0.6);
                sleep(250);
                encoderDrive(0.4, 20,4, false, 2, true, false, 0);
            } else if(goldMineralPosition == 3){
                autoSwerve.spinPosition();
                sleep(250);
                gyroTurn(0.4, -45, 0.011);
                sleep(250);
                autoSwerve.straightPosition();
                sleep(250);
                encoderDrive(0.4, 20,4, false, 2, true, false, 0);
                //sleep(250);
                //autoSwerve.rightKnock();
            } else if(goldMineralPosition == 0){
                encoderDrive(0.4, 20,4, false, 2, true, false, 0);
            }

            stop();
            telemetry.update();

            }
    }

    public  void webcamScan(){
        while(opModeIsActive()){
            autoSwerve.WebcamPan.setPosition(0.75);
            sleep(1000);
            if(detector.getAligned()){
                telemetry.addLine("left");
                telemetry.update();
                goldMineralPosition = 1;
                sleep(500);
                break;
            }
            else{
                autoSwerve.WebcamPan.setPosition(0.5);
            }
            sleep(1000);
            if(detector.getAligned()){
                telemetry.addLine("center");
                telemetry.update();
                goldMineralPosition = 2;
                sleep(500);
                break;
            }
            else{
                autoSwerve.WebcamPan.setPosition(0.29);
            }
            sleep(1000);
            if(detector.getAligned()){
                telemetry.addLine("right");
                telemetry.update();
                goldMineralPosition = 3;
                sleep(500);
                break;
            }
            else{
                telemetry.addLine("Defalt");
                telemetry.update();
                goldMineralPosition = 0;
            }
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
            newLFTarget = autoSwerve.SwervePod3motor.getCurrentPosition() + (int) (leftDistance * autoLibrary.COUNTS_PER_INCH);
            newLRTarget = autoSwerve.SwervePod4motor.getCurrentPosition() + (int) (leftDistance * autoLibrary.COUNTS_PER_INCH);
            newRFTarget = autoSwerve.SwervePod1motor.getCurrentPosition() + (int) (rightDistance * autoLibrary.COUNTS_PER_INCH);
            newRRTarget = autoSwerve.SwervePod2motor.getCurrentPosition() + (int) (rightDistance * autoLibrary.COUNTS_PER_INCH);

            while (autoSwerve.SwervePod3motor.getTargetPosition() != newLFTarget) {
                autoSwerve.SwervePod3motor.setTargetPosition(newLFTarget);
                sleep(1);
            }
            while (autoSwerve.SwervePod1motor.getTargetPosition() != newRFTarget) {
                autoSwerve.SwervePod1motor.setTargetPosition(newRFTarget);
                sleep(1);
            }
            while (autoSwerve.SwervePod4motor.getTargetPosition() != newLRTarget) {
                autoSwerve.SwervePod4motor.setTargetPosition(newLRTarget);
                sleep(1);
            }
            while (autoSwerve.SwervePod2motor.getTargetPosition() != newRRTarget) {
                autoSwerve.SwervePod2motor.setTargetPosition(newRRTarget);
                sleep(1);
            }

            // Turn On motors to RUN_TO_POSITION
            autoSwerve.SwervePod3motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoSwerve.SwervePod4motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoSwerve.SwervePod1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoSwerve.SwervePod2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            autoLibrary.runtime.reset();

            speed = Math.abs(speed);    // Make sure its positive
            curSpeed = Math.min(MINSPEED, speed);

            // Set the motors to the starting power
            autoSwerve.SwervePod3motor.setPower(Math.abs(curSpeed));
            autoSwerve.SwervePod1motor.setPower(Math.abs(curSpeed));
            autoSwerve.SwervePod4motor.setPower(Math.abs(curSpeed));
            autoSwerve.SwervePod2motor.setPower(Math.abs(curSpeed));

            // keep looping while we are still active, and there is time left, until at least 1 motor reaches target
            while (opModeIsActive() &&
                    (autoLibrary.runtime.seconds() < timeout) &&
                    autoSwerve.SwervePod3motor.isBusy() &&
                    autoSwerve.SwervePod4motor.isBusy() &&
                    autoSwerve.SwervePod1motor.isBusy() &&
                    autoSwerve.SwervePod2motor.isBusy()) {

                // Ramp up motor powers as needed
                if (curSpeed < speed) {
                    curSpeed += SPEEDINCR;
                }
                double leftSpeed = curSpeed;
                double rightSpeed = curSpeed;

                // Doing gyro heading correction?
                if (useGyro) {

                    // adjust relative speed based on heading
                    double error = getError(curHeading);

                    updateGyroErrorAvg(error);

                    double steer = getSteer(error,
                            (aggressive ? P_DRIVE_COEFF_2 : P_DRIVE_COEFF_1));

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    // Adjust motor powers for heading correction
                    leftSpeed -= steer;
                    rightSpeed += steer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                }

                // And rewrite the motor speeds
                autoSwerve.SwervePod3motor.setPower(Math.abs(leftSpeed));
                autoSwerve.SwervePod1motor.setPower(Math.abs(rightSpeed));
                autoSwerve.SwervePod4motor.setPower(Math.abs(leftSpeed));
                autoSwerve.SwervePod2motor.setPower(Math.abs(rightSpeed));

                // Allow time for other processes to run.
                sleep(1);
                ;
            }


            RobotLog.i("DM10337- encoderDrive done" +
                    "  lftarget: " + newLFTarget + "  lfactual:" + autoSwerve.SwervePod3motor.getCurrentPosition() +
                    "  lrtarget: " + newLFTarget + "  lractual:" + autoSwerve.SwervePod4motor.getCurrentPosition() +
                    "  rftarget: " + newRFTarget + "  rfactual:" + autoSwerve.SwervePod1motor.getCurrentPosition() +
                    "  rrtarget: " + newRFTarget + "  rractual:" + autoSwerve.SwervePod2motor.getCurrentPosition() +
                    "  heading:" + readGyro());

            RobotLog.i("DM10337 - Gyro error average: " + gyroErrorAvg.average());

            // Stop all motion;
            autoSwerve.SwervePod3motor.setPower(0);
            autoSwerve.SwervePod4motor.setPower(0);
            autoSwerve.SwervePod1motor.setPower(0);
            autoSwerve.SwervePod2motor.setPower(0);

            // Turn off RUN_TO_POSITION
            autoSwerve.SwervePod3motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoSwerve.SwervePod4motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoSwerve.SwervePod1motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            autoSwerve.SwervePod2motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void updateGyroErrorAvg(double error) {
        gyroErrorAvg.add(Math.abs(error));
    }

    public void gyroTurn(double speed, double angle, double coefficient) {

        telemetry.addLine("DM10337- gyroTurn start  speed:" + speed +
                "  heading:" + angle);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, coefficient)) {
            // Allow time for other processes to run.
            // onHeading() does the work of turning us
            sleep(1);
            ;
        }

        telemetry.addLine("DM10337- gyroTurn done   heading actual:" + readGyro());
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        int HEADING_THRESHOLD = 5;
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            // Close enough so no need to move
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            // Calculate motor powers
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        autoSwerve.SwervePod3motor.setPower(leftSpeed);
        autoSwerve.SwervePod1motor.setPower(rightSpeed);
        autoSwerve.SwervePod4motor.setPower(leftSpeed);
        autoSwerve.SwervePod2motor.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        autoLibrary.angles = autoLibrary.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - autoLibrary.angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     * Record the current heading and use that as the 0 heading point for gyro reads
     *
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
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                autoLibrary.angles = autoLibrary.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                autoLibrary.gravity = autoLibrary.imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return autoLibrary.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return autoLibrary.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(autoLibrary.angles.angleUnit, autoLibrary.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(autoLibrary.angles.angleUnit, autoLibrary.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(autoLibrary.angles.angleUnit, autoLibrary.angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return autoLibrary.gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(autoLibrary.gravity.xAccel * autoLibrary.gravity.xAccel
                                        + autoLibrary.gravity.yAccel * autoLibrary.gravity.yAccel
                                        + autoLibrary.gravity.zAccel * autoLibrary.gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
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
}
