package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Swerve_Library extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public Servo SwervePod1 = null;
    public Servo SwervePod2 = null;
    public Servo SwervePod3 = null;
    public Servo SwervePod4 = null;
    public Servo WebcamPan = null;
    public CRServo ScoreFlipper = null;
    public Servo cubeKnockRight = null;
    public  Servo cubeKnockLeft = null;
    public CRServo intakeLift = null;
    public Servo intakeRightLift = null;
    public Servo intakeLeftLift = null;
    public DcMotor IntakeLiftRight = null;
    public DcMotor IntakeLiftLeft = null;
    public DcMotor SwervePod1motor = null;
    public DcMotor SwervePod2motor = null;
    public DcMotor SwervePod3motor = null;
    public DcMotor SwervePod4motor = null;
    public DcMotor IntakeMotor = null;
    public DcMotor LiftMotor = null;

    private static final double[] Latch = new double[]{0, 1};
    private int currentLatchIndex;

    HardwareMap hardwareMap           =  null;


    @Override
    public void runOpMode(){

    }

    public void init(HardwareMap ahwMap){
        hardwareMap = ahwMap;

        this.currentLatchIndex = 0;

        SwervePod1 = hardwareMap.get(Servo.class, "Swerve_Pod1");
        SwervePod2 = hardwareMap.get(Servo.class, "Swerve_Pod2");
        SwervePod3 = hardwareMap.get(Servo.class, "Swerve_Pod3");
        SwervePod4 = hardwareMap.get(Servo.class, "Swerve_Pod4");
        cubeKnockLeft = hardwareMap.get(Servo.class, "knockleft");
        cubeKnockRight = hardwareMap.get(Servo.class, "knockright");
       // ScoreFlipper = hardwareMap.get(Servo.class, "ScoreFlipper");
        intakeLift = hardwareMap.get(CRServo.class, "IntakeLift");
        intakeRightLift = hardwareMap.get(Servo.class, "IntakeRightLift");
        intakeLeftLift = hardwareMap.get(Servo.class, "IntakeLeftLift");
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
        IntakeLiftRight.setDirection(DcMotor.Direction.FORWARD);
        IntakeLiftLeft.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        LiftMotor.setDirection(DcMotor.Direction.REVERSE);

        SwervePod1motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod2motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod3motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SwervePod4motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SwervePod1motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod2motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod3motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SwervePod4motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (currentLatchIndex == 0) {
            SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
            SwervePod2motor.setDirection(DcMotor.Direction.REVERSE);
            SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
            SwervePod4motor.setDirection(DcMotor.Direction.FORWARD);
        } else if (currentLatchIndex == 1) {
            SwervePod1motor.setDirection(DcMotor.Direction.REVERSE);
            SwervePod2motor.setDirection(DcMotor.Direction.FORWARD);//changed
            SwervePod3motor.setDirection(DcMotor.Direction.FORWARD);
            SwervePod4motor.setDirection(DcMotor.Direction.REVERSE);//changed
        }

        double SwervePod1Position = 0.5;
        double SwervePod2Position = 0.5;
        double SwervePod3Position = 0.5;
        double SwervePod4Position = 0.5;
        double CubeKnockLeftPosition = 0.92;
        double CubeKnockRightPosition = 0.13;
        double intakeRightLiftPosition = 1.0;
        double intakeLeftLiftPosition = 0.0;
        double ScoreFlipperPosition = 0.2;
       // ScoreFlipper.setPosition(ScoreFlipperPosition);
        SwervePod2.setPosition(SwervePod2Position);
        SwervePod1.setPosition(SwervePod1Position);
        SwervePod3.setPosition(SwervePod3Position);
        SwervePod4.setPosition(SwervePod4Position);
        cubeKnockRight.setPosition(CubeKnockRightPosition);
        cubeKnockLeft.setPosition(CubeKnockLeftPosition);
        intakeLeftLift.setPosition(intakeLeftLiftPosition);
        intakeRightLift.setPosition(intakeRightLiftPosition);
        //IntakeSlide.setPosition(IntakePosition);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

    }
    public void straightPosition(){
        SwervePod1.setPosition(0.5);
        SwervePod2.setPosition(0.5);
        SwervePod3.setPosition(0.5);
        SwervePod4.setPosition(0.5);
        currentLatchIndex = 0 % Latch.length;
    }
    public void turnPosition(){
        SwervePod1.setPosition(0.8);
        SwervePod2.setPosition(0.2);
        SwervePod3.setPosition(0.8);
        SwervePod4.setPosition(0.2);
        currentLatchIndex = 1 % Latch.length;
    }
    public void spinPosition(){
        SwervePod1.setPosition(0.68);
        SwervePod2.setPosition(0.35);
        SwervePod3.setPosition(0.68);
        SwervePod4.setPosition(0.35);
        currentLatchIndex = 0 % Latch.length;
    }
    public void right45Position(){
        SwervePod1.setPosition(0.35);
        SwervePod2.setPosition(0.35);
        SwervePod3.setPosition(0.35);
        SwervePod4.setPosition(0.35);
        currentLatchIndex = 0 % Latch.length;
    }
    public void left45Position(){
        SwervePod1.setPosition(0.685);
        SwervePod2.setPosition(0.68);
        SwervePod3.setPosition(0.68);
        SwervePod4.setPosition(0.685);
        currentLatchIndex = 0 % Latch.length;
    }

}
