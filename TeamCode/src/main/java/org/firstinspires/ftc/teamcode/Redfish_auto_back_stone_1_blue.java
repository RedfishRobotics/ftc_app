/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*This is a test is the REV Color Sensor for game use.
 **The test will be getting the color reading and then using the reading to activate our grippers.
 **In the real program the pushers wouldn't be used,but from this test we are only testing the logic of the code
 */
@Autonomous(name = "Redfish_auto_back_stone_blue", group = "Sensor")

public class Redfish_auto_back_stone_1_blue extends LinearOpMode {

    // Declare Opmode members.
    VuforiaLocalizer vuforia;
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
    private Servo IntakeStop = null;
    private Servo ColorArmTurn = null;
    private Servo ColorSensorArm = null;
    private CRServo PITA_1 = null;
    private CRServo PITA_2 = null;
    private Servo glyphStop = null;
    private Servo relicPivot = null;
    private ColorSensor sensorColor;
    private BNO055IMU imu;
    private OpenGLMatrix lastLocation = null;

    private Orientation angles;
    private Acceleration gravity;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // The encoder ticks per revolution for andymark 40 motors
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // The gear reduction on our motors
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // The diameter to get the circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);// The math to get the number of inches per rotation of the motors


    @Override
    public void runOpMode() {


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";


        // set references for config

        leftFrontDrive =  hardwareMap.get(DcMotor.class, "Left_FM");
        leftRearDrive =  hardwareMap.get(DcMotor.class, "Left_RM");
        rightFrontDrive =  hardwareMap.get(DcMotor.class, "Right_FM");
        rightRearDrive =  hardwareMap.get(DcMotor.class, "Right_RM");
        RightLiftMotor = hardwareMap.get(DcMotor.class, "RightLiftMotor");
        LeftLiftMotor = hardwareMap.get(DcMotor.class, "LeftLiftMotor");
        RightIntakeMotor = hardwareMap.get(DcMotor.class, "RightIntakeMotor");
        LeftIntakeMotor = hardwareMap.get(DcMotor.class, "LeftIntakeMotor");
        rightFlip = hardwareMap.get(Servo.class, "Right_Flip");
        leftFlip = hardwareMap.get(Servo.class, "Left_Flip");
        PITA_1 = hardwareMap.get(CRServo.class, "Intake_Stop_Right");
        PITA_2 = hardwareMap.get(CRServo.class, "Intake_Stop_Left");
        ColorArmTurn = hardwareMap.get(Servo.class, "colorArmTurn");
        ColorSensorArm = hardwareMap.get(Servo.class , "colorSensorArm");
        relicPivot = hardwareMap.get(Servo.class, "Relic_Pivot");
        glyphStop = hardwareMap.get(Servo.class, "Glyph_Stop");
        ColorArmTurn = hardwareMap.get(Servo.class, "colorArmTurn");
        ColorSensorArm = hardwareMap.get(Servo.class , "colorSensorArm");

        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");

        // set the direction of the motor
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        RightLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        RightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        LeftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rightFlipPosition = 1.0;
        double IntakeStopPosition = 0.05;
        double leftFlipPosition = 0.0;
        double colorArmTurnPosition = 0.88;
        double colorSensorPosition = 0.77;


        ColorArmTurn.setPosition(colorArmTurnPosition);
        ColorSensorArm.setPosition(colorSensorPosition);
        leftFlip.setPosition(leftFlipPosition);
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVuforia = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        double TurnDegrees = 0.0;

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parametersVuforia.vuforiaLicenseKey = "AVg/GH7/////AAAAGXXrrHEuaEOzp854t67yt2t4YfSS5Iyu/3WJ5LWSNOtgL5mU1w7vaWX+xkMfu5KrjEezUhS3ZjmDjGN/kWJv4vPYUHhZEXGN1U0TCHGs4OFyqoRvtmzMCSe8QUgFu4QqTc/HtCS1p9GomGMP0x2uicjUbtrZmYEQsrHF58G/UzKB0WFqQpq0bQUIAvSprFLK2/KhqxvJ5YbBjlDGTjVCbiGbYCNoRSMrf/BQZXbIoIT4J7bW+ciKShWTc50Ixq6dSJAvM4a8KAO3HnAw2tvUFPk024VUNSOpa5aT+31U1hmhhb+UM4z2KxrGvnVtxgVbQlPiIOYdo6fsKt657wECa4lzztFKkKvWyDzJqDRZf+va\n";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parametersVuforia.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT  ;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parametersVuforia);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();




        // hsvValues is an array that will hold the hue, saturation, and value information.
        // set the init position for the servo
        // int HEADING_THRESHOLD = 5;
        //double headingBias;
        // set servos to those postions
        // wait for the start button to be pressed.
        composeTelemetry();

        telemetry.update();
        waitForStart();
        relicTrackables.activate();
        relicPivot.setPosition(0.95);
        sleep(250);
        ColorArmTurn.setPosition(0.59);
        sleep(500);
        ColorSensorArm.setPosition(0.15);
        sleep(1000);
        while(opModeIsActive()){
            ColorArmTurn.setPosition(0.59);
            sleep(500);
            ColorSensorArm.setPosition(0.15);
            sleep(1000);
            if(sensorColor.red() < sensorColor.blue()){//If the sensor sees blue
                jewelKnockLeft(250);
                sleep(250);
                ColorSensorArm.setPosition(0.77);
                sleep(250);
                ColorArmTurn.setPosition(0.88);
            }
            else if(sensorColor.blue() < sensorColor.red()){//If the sensor sees red
                jewelKnockRight(250);
                sleep(250);
                ColorSensorArm.setPosition(0.77);
                sleep(250);
                ColorArmTurn.setPosition(0.88);
            }
            else{
                ColorSensorArm.setPosition(0.77);
                sleep(250);
                ColorArmTurn.setPosition(0.88);
            }
            sleep(1000);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            // if (vuMark.equals(RelicRecoveryVuMark.CENTER)){
            //             encoderDrive(0.5, 14, 14, 5);
            //             sleep(500);
            //             gyroTurn(0.5, 180, 0.009);
            //             sleep(500);
            //             encoderStrafeLeft(0.5, 14, 14, 5);
            //             telemetry.addLine("Center");
            //         }
            //         else if (vuMark.equals(RelicRecoveryVuMark.LEFT)){
            //             encoderDrive(0.5, 14, 14, 5);
            //             sleep(500);
            //     gyroTurn(0.5, 180, 0.009);
            //     sleep(500);
            //     encoderStrafeLeft(0.5, 7, 7, 5);
            //     telemetry.addLine("Left");
            // }
            // else if (vuMark.equals(RelicRecoveryVuMark.RIGHT)){
            //   encoderDrive(0.5, 14, 14, 5);
            //   sleep(500);
            //   gyroTurn(0.5, 180, 0.009);
            //   sleep(500);
            //   encoderStrafeLeft(0.5, 21, 21, 5);
            //   telemetry.addLine("Right");
            // }

            encoderDrive(0.5, 10, 10, 5);
            sleep(500);
            gyroTurn(0.5, 180, 0.009);
            sleep(500);
            encoderStrafeLeft(0.5, 10, 10, 5);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(250);
            encoderDrive(0.5, -4, -4, 5);
            sleep(200);
            intakeDown();
            sleep(250);
            glyphStop.setPosition(1.0);
            sleep(350);
            glyphStop.setPosition(0.85);
            sleep(250);
            leftFlip.setPosition(0.15);
            rightFlip.setPosition(0.85);
            sleep(750);
            leftFlip.setPosition(0.65);
            rightFlip.setPosition(0.35);
            sleep(500);
            leftFlip.setPosition(0.0);
            rightFlip.setPosition(1.0);
            sleep(500);
            glyphStop.setPosition(0.3);
            sleep(500);
            encoderDrive(0.5, 4, 4, 5);
            telemetry.update();
            stop();
        }
    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
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
            leftSpeed  = speed * -steer;
            rightSpeed   = -leftSpeed;
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
            leftFrontDrive.setPower(Math.abs(speed));
            leftRearDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightRearDrive.setPower(Math.abs(speed));

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
            leftFrontDrive.setPower(Math.abs(speed));
            leftRearDrive.setPower(-Math.abs(speed));
            rightFrontDrive.setPower(-Math.abs(speed));
            rightRearDrive.setPower(Math.abs(speed));

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
            leftFrontDrive.setPower(-Math.abs(speed));
            leftRearDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightRearDrive.setPower(-Math.abs(speed));

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
    void lift(int target){
        //Set the arm motors new target position
        RightLiftMotor.setTargetPosition(target);
        LeftLiftMotor.setTargetPosition(target);
        //Turn on RUN_TO_POSITION
        LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && RightLiftMotor.isBusy()){
            if(target > RightLiftMotor.getCurrentPosition()){//If the target is below the current value, set motor power to 20%
                RightLiftMotor.setPower(0.2);
                LeftLiftMotor.setPower(0.2);
            }
            else if(target < RightLiftMotor.getCurrentPosition()){//If the target is above the current value, set motor power to 25%
                RightLiftMotor.setPower(0.25);
                LeftLiftMotor.setPower(0.25);
            }
        }
    }
    void jewelKnockRight(int sleepPerTurn){
        double colorArmTurnPosition = 0.62;
        while(ColorArmTurn.getPosition() < 0.8){
            colorArmTurnPosition += 0.02;
            ColorArmTurn.setPosition(colorArmTurnPosition);
            sleep(sleepPerTurn);
        }
    }
    void jewelKnockLeft(int sleepPerTurn){
        double colorArmTurnPosition = 0.62;
        while(ColorArmTurn.getPosition() > 0.4){
            colorArmTurnPosition -= 0.02;
            ColorArmTurn.setPosition(colorArmTurnPosition);
            sleep(sleepPerTurn);
        }
    }
    void IntakeStopDown(int sleepPerTurn){
        double IntakeStopPosition = 0.05;
        while(IntakeStop.getPosition() < 0.8){
            IntakeStopPosition += 0.03;
            IntakeStop.setPosition(IntakeStopPosition);
            sleep(sleepPerTurn);
        }
    }
    void intakeDown(){
        PITA_1.setPower(0.8);
        PITA_2.setPower(-0.8);
        sleep(2500);
        PITA_1.setPower(0);
        PITA_2.setPower(0);
    }




}
