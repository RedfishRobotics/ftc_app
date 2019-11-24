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


public class Auto_Library_Rover_Ruckus extends LinearOpMode {

    VuforiaLocalizer vuforia;
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor hanger = null;
//    public CRServo intake = null;

    public DigitalChannel magneticSwitchStaging = null;
    public CRServo boxServo = null;
    public CRServo boxArm = null;
    public Servo panServo = null;
    public DcMotor rightRearDrive = null;
//    public DcMotor intakeMotor = null;
//    public DcMotor intakeSlideMotor = null;
//    public DcMotor rightLifter = null;
    public DcMotor leftLifter = null;
    public CRServo leftSlide = null;
    public CRServo rightSlide = null;


    public BNO055IMU imu;
    public OpenGLMatrix lastLocation = null;

    public Orientation angles;
    public Acceleration gravity;

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // The encoder ticks per revolution for andymark 40 motors
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // The gear reduction on our motors
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
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_FM");
        leftRearDrive = hardwareMap.get(DcMotor.class, "Left_RM");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_FM");
        rightRearDrive = hardwareMap.get(DcMotor.class, "Right_RM");
        magneticSwitchStaging = hardwareMap.get(DigitalChannel.class, "Stage_Hull_Effect");
        boxArm = hardwareMap.get(CRServo.class, "Box_Arm");
        boxServo = hardwareMap.get(CRServo.class, "Box_Servo");
//        rightLifter = hardwareMap.get(DcMotor.class, "Right_Lifter");
//        leftLifter = hardwareMap.get(DcMotor.class, "Left_Lifter");
//        intakeMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        leftSlide = hardwareMap.get(CRServo.class, "Left_Slide");
        rightSlide = hardwareMap.get(CRServo.class, "Right_Slide");
//        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Slide_Motor");
        panServo = hardwareMap.get(Servo.class, "Pan_Servo");
        hanger = hardwareMap.get(DcMotor.class, "Hanger");
//        intake = hardwareMap.get(CRServo.class, "Intake_Servo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        hanger.setDirection(DcMotor.Direction.REVERSE);
//        rightLifter.setDirection(DcMotor.Direction.FORWARD);
//        leftLifter.setDirection(DcMotor.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(CRServo.Direction.FORWARD);
        rightSlide.setDirection(CRServo.Direction.FORWARD);
//        intakeSlideMotor.setDirection(DcMotor.Direction.FORWARD);

//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        panServo.setPosition(0.6);
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
        leftFrontDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        leftRearDrive.setPower(leftSpeed);
        rightRearDrive.setPower(rightSpeed);

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
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftTarget);
            leftRearDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            rightRearDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(speed);
            leftRearDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightRearDrive.setPower(speed);

            //While the motors and OpMode is running, return telemetry on the motors
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftTarget);
            leftRearDrive.setTargetPosition(-newLeftTarget);
            rightFrontDrive.setTargetPosition(-newRightTarget);
            rightRearDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(speed);
            leftRearDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightRearDrive.setPower(speed);

            //While the motors and OpMode is running, return telemetry on the motors
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(-newLeftTarget);
            leftRearDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            rightRearDrive.setTargetPosition(-newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(speed);
            leftRearDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightRearDrive.setPower(speed);


            //While the motors and OpMode is running, return telemetry on the motors
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void hangerUp(){
            Hanger(17700);
    }
    public void hangerHalf(){
        Hanger(8850);
    }
    public void hangerDown(){
        Hanger(10);
    }
    public void Hanger(int target) {
        //Sets the new target position for the glyph lifter
        // RightLiftMotor.setTargetPosition(target);
        hanger.setTargetPosition(target);
        //Turns on RUN_TO_POSITION
        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //If statement that ask if the motor is busy
        if (hanger.isBusy()) {
            //If statement that checks if the motors current position is more then the target
            if (hanger.getCurrentPosition() > target) {
                //If the current position is more than the target, set motor power to 40%
                // RightLiftMotor.setPower(0.45);
                hanger.setPower(0.65);
            }
            //If statement that checks if the motors current position is less then the target
            else if (hanger.getCurrentPosition() < target) {
                //If the current position is more than the target, set motor power to 60%
                // RightLiftMotor.setPower(0.5);
                hanger.setPower(0.65);

            }
        }
    }
//    public void intakeStop(){
//        intakeMotor.setPower(0.0);
//    }
//    public void intakeIn(){
//        intakeMotor.setPower(0.8);
//    }
//    public void intakeOut(){
//        intakeMotor.setPower(-0.4);
//    }
//    public void lifterTucked(){
//        liftTestLeft(0);
//        liftTestRight(0);
//    }
//    public void lifterScoring(){
//        liftTestLeft(775);
//        liftTestRight(550);
//    }
//    public void lifterDown(){
//        liftTestLeft(1750);
//        liftTestRight(1650);
//    }
//    public void liftTestRight(int target){
//        //Sets the new target position for the glyph lifter
//        // RightLiftMotor.setTargetPosition(target);
//        rightLifter.setTargetPosition(target);
//        //Turns on RUN_TO_POSITION
//        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //If statement that ask if the motor is busy
//        if( rightLifter.isBusy()){
//            //If statement that checks if the motors current position is more then the target
//            if( rightLifter.getCurrentPosition() > target){
//                //If the current position is more than the target, set motor power to 40%
//                // RightLiftMotor.setPower(0.45);
//                rightLifter.setPower(0.4);
//            }
//            //If statement that checks if the motors current position is less then the target
//            else if( rightLifter.getCurrentPosition() < target){
//                //If the current position is more than the target, set motor power to 60%
//                // RightLiftMotor.setPower(0.5);
//                rightLifter.setPower(0.15);
//
//            }
//        }
//    }
//    public void liftTestLeft(int target){
//        //Sets the new target position for the glyph lifter
//        // RightLiftMotor.setTargetPosition(target);
//        leftLifter.setTargetPosition(target);
//        //Turns on RUN_TO_POSITION
//        // RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //If statement that ask if the motor is busy
//        if( leftLifter.isBusy()){
//            //If statement that checks if the motors current position is more then the target
//            if( leftLifter.getCurrentPosition() > target){
//                //If the current position is more than the target, set motor power to 40%
//                // RightLiftMotor.setPower(0.45);
//                leftLifter.setPower(0.4);
//            }
//            //If statement that checks if the motors current position is less then the target
//            else if( leftLifter.getCurrentPosition() < target){
//                //If the current position is more than the target, set motor power to 60%
//                // RightLiftMotor.setPower(0.5);
//                leftLifter.setPower(0.15);
//            }
//        }
//    }
}
