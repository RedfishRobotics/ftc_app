package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;


public class Auto_Library_Swerve extends LinearOpMode {

    VuforiaLocalizer vuforia;
    Servo SwervePod1 = null;
    Servo SwervePod2 = null;
    Servo SwervePod3 = null;
    Servo SwervePod4 = null;
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
    Servo WebcamPan = null;
    public ElapsedTime runtime = new ElapsedTime();

    public BNO055IMU imu;
    public OpenGLMatrix lastLocation = null;

    public Orientation angles;
    public Acceleration gravity;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // The encoder ticks per revolution for andymark 40 motors
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // The gear reduction on our motors
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // The diameter to get the circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);// The math to get the number of inches per rotation of the motors


    HardwareMap hardwareMap           =  null;

    @Override
    public void runOpMode(){

    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        // Define and Initialize Motors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";


        // set references for config

        SwervePod1 = hardwareMap.get(Servo.class, "Swerve_Pod1");
        SwervePod2 = hardwareMap.get(Servo.class, "Swerve_Pod2");
        SwervePod3 = hardwareMap.get(Servo.class, "Swerve_Pod3");
        SwervePod4 = hardwareMap.get(Servo.class, "Swerve_Pod4");
        cubeKnockLeft = hardwareMap.get(Servo.class, "knockleft");
        cubeKnockRight = hardwareMap.get(Servo.class, "knockright");
        WebcamPan = hardwareMap.get(Servo.class, "webcamPan");
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
        IntakeLiftLeft.setDirection(DcMotor.Direction.REVERSE);
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
        SwervePod2.setPosition(SwervePod2Position);
        SwervePod1.setPosition(SwervePod1Position);
        SwervePod3.setPosition(SwervePod3Position);
        SwervePod4.setPosition(SwervePod4Position);
        cubeKnockRight.setPosition(CubeKnockRightPosition);
        cubeKnockLeft.setPosition(CubeKnockLeftPosition);

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);

    }
    public void turnPosition(){
        SwervePod1.setPosition(0.8);
        SwervePod2.setPosition(0.2);
        SwervePod3.setPosition(0.8);
        SwervePod4.setPosition(0.2);
        sleep(250);
        SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod2motor.setDirection(DcMotor.Direction.FORWARD);//changed
        SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
        SwervePod4motor.setDirection(DcMotor.Direction.REVERSE);//changed
    }
    public void left45Position(){
        SwervePod1.setPosition(0.35);
        SwervePod2.setPosition(0.35);
        SwervePod3.setPosition(0.35);
        SwervePod4.setPosition(0.35);
        sleep(250);
        SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
        SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void right45Position(){
        SwervePod1.setPosition(0.685);
        SwervePod2.setPosition(0.68);
        SwervePod3.setPosition(0.68);
        SwervePod4.setPosition(0.685);
        sleep(250);
        SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
        SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void spinPosition(){
        SwervePod1.setPosition(0.68);
        SwervePod2.setPosition(0.35);
        SwervePod3.setPosition(0.68);
        SwervePod4.setPosition(0.35);
        sleep(250);
        SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
        SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void straightPosition(){
        SwervePod1.setPosition(0.5);
        SwervePod2.setPosition(0.5);
        SwervePod3.setPosition(0.5);
        SwervePod4.setPosition(0.5);
        sleep(250);
        SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
        SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
        SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void rightKnock(){
        cubeKnockRight.setPosition(0.5);
    }
    public void leftKnock(){
        cubeKnockLeft.setPosition(0.55);
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
        SwervePod3motor.setPower(leftSpeed);
        SwervePod1motor.setPower(rightSpeed);
        SwervePod4motor.setPower(leftSpeed);
        SwervePod2motor.setPower(rightSpeed);

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
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
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
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingBias = angles.firstAngle;
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
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
        double headingBias = angles.firstAngle;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angles.firstAngle - headingBias;
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;//init the variable newLeftTarget
        int newRightTarget;//init the variable newRightTarget

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = SwervePod3motor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = SwervePod1motor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            SwervePod3motor.setTargetPosition(newLeftTarget);
            SwervePod4motor.setTargetPosition(newLeftTarget);
            SwervePod1motor.setTargetPosition(newRightTarget);
            SwervePod2motor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            SwervePod3motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SwervePod4motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SwervePod1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            SwervePod2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            SwervePod3motor.setPower(speed);
            SwervePod4motor.setPower(speed);
            SwervePod1motor.setPower(speed);
            SwervePod2motor.setPower(speed);

            //While the motors and OpMode is running, return telemetry on the motors
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (SwervePod3motor.isBusy() && SwervePod1motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        SwervePod3motor.getCurrentPosition(),
                        SwervePod1motor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            SwervePod3motor.setPower(0);
            SwervePod4motor.setPower(0);
            SwervePod1motor.setPower(0);
            SwervePod2motor.setPower(0);

            // Turn off RUN_TO_POSITION
            SwervePod3motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SwervePod4motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SwervePod1motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SwervePod2motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void runIntakeForward() {
        IntakeMotor.setPower(0.8);
    }
    public void stopIntake() {
        IntakeMotor.setPower(0);
    }
    public void runIntakeHalf() {
        IntakeMotor.setPower(0.6);
    }
    public void runIntakeBackward() {
        IntakeMotor.setPower(-0.55);
    }
}
