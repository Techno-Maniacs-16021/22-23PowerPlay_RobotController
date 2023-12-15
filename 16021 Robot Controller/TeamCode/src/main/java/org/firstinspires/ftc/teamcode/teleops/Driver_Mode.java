package org.firstinspires.ftc.teamcode.teleops;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.widget.HorizontalScrollView;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.RobotIntialization;

@Config
@TeleOp
public class Driver_Mode extends OpMode
{
    /////////////////////////////////////////////
    ServoImplEx left_arm, right_arm, wrist;
    CRServo left_intake, right_intake;
    DcMotorEx horizontal_slides, left_vertical_slides,right_vertical_slides;
    AnalogInput rightArmPosition,leftArmPosition;
    RevColorSensorV3 cone_detector;
    /////////////////////////////////////////////
    private ElapsedTime armCooldown = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();
    private SampleMecanumDrive drive;
    private RobotIntialization robotIntialization;
    private PIDController hController,vController;
    public static double hp,hi,hd,hTarget;
    public static double vp,vi,vd,vf,vTarget;
    public static boolean farmingMode = true;
    public static boolean active = true;
    public static double armPosition,wristPosition,intakePower,speedMultiplier;
    boolean intaked,outaked,OVERIDE;
    public static int HIGH,MID;
    /////////////////////////////////////////////
    @Override
    public void init(){
////////////////////////HARDWARE INIT////////////////
        drive = new SampleMecanumDrive(hardwareMap);
        left_arm = hardwareMap.get(ServoImplEx.class, "LA");
        right_arm = hardwareMap.get(ServoImplEx.class, "RA");
        wrist = hardwareMap.get(ServoImplEx.class,"Wrist");
        left_intake = hardwareMap.get(CRServo.class,"LI");
        right_intake = hardwareMap.get(CRServo.class,"RI");
        rightArmPosition = hardwareMap.get(AnalogInput.class,"rArmPos");
        leftArmPosition = hardwareMap.get(AnalogInput.class,"lArmPos");
        cone_detector = hardwareMap.get(RevColorSensorV3.class,"coneDetector");
        left_vertical_slides = hardwareMap.get(DcMotorEx.class,"LV");
        right_vertical_slides = hardwareMap.get(DcMotorEx.class,"RV");
        horizontal_slides = hardwareMap.get(DcMotorEx.class,"H");
////////////////////////SET PWM RANGE////////////////
        left_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        right_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        wrist.setPwmRange(new PwmControl.PwmRange(510,2490));
////////////////////////HARDWARE REVERSING///////////
        right_arm.setDirection(ServoImplEx.Direction.REVERSE);
        left_intake.setDirection(CRServo.Direction.REVERSE);
        left_vertical_slides.setDirection(DcMotorSimple.Direction.REVERSE);
////////////////////////MOTOR BRAKE BEHAVIOR/////////
        left_vertical_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_vertical_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        horizontal_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////////////////////////ENCODER RESET////////////////
        left_vertical_slides.setMode(RUN_WITHOUT_ENCODER);
        right_vertical_slides.setMode(RUN_WITHOUT_ENCODER);
        horizontal_slides.setMode(RUN_WITHOUT_ENCODER);
////////////////////////PID CONTROLLERS//////////////
        hController = new PIDController(hp,hi,hd);
        vController = new PIDController(vp,vi,vd);
////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////DRIVE INIT///////////////////
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        vp=0.011;vi=0;vd=0.00005;vf=0.015;vTarget = 0;hp=0.0035;hi=0;hd=0.000;hTarget = 0;speedMultiplier=1;
        armPosition = 0.5;wristPosition = 0;intakePower=0;
        HIGH = 930;MID= 00;
        intaked = false; outaked=false; farmingMode = false; OVERIDE = false; active=true;
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        left_arm.setPosition(armPosition);
        right_arm.setPosition(armPosition);
        wrist.setPosition(wristPosition);
        left_intake.setPower(intakePower);
        right_intake.setPower(intakePower);
    }
    @Override
    public void loop(){
////////////////////////HORIZONTAL SLIDES PID////////
        hController.setPID(hp,hi,hd);
        int hPos = horizontal_slides.getCurrentPosition();
        double hPID = hController.calculate(hPos,hTarget);
        double hPower = hPID;
        if(!gamepad2.b)horizontal_slides.setPower(hPower);
////////////////////////VERTICAL SLIDES PID//////////
        vController.setPID(vp,vi,vd);
        int vPos = (right_vertical_slides.getCurrentPosition()+left_vertical_slides.getCurrentPosition())/2;
        double vPID = vController.calculate(vPos,vTarget);
        double vPower = vPID+vf;
        if(!gamepad2.b){
            left_vertical_slides.setPower(-verticalPID(vTarget, vController, left_vertical_slides.getCurrentPosition()));
            right_vertical_slides.setPower(verticalPID(vTarget, vController, right_vertical_slides.getCurrentPosition()));
        }
////////////////////////ARM ANG VARS/////////////////
        double leftArmAngle = ((leftArmPosition.getVoltage()-0.82) /3.3) * 360;
        double rightArmAngle = ((2.499-rightArmPosition.getVoltage())/3.3) * 360;
        double armAngle = (leftArmAngle+rightArmAngle)/2;
////////////////////////SERVO POSITIONS//////////////
        left_arm.setPosition(armPosition);
        right_arm.setPosition(armPosition);
        wrist.setPosition(wristPosition);
        left_intake.setPower(intakePower);
        right_intake.setPower(intakePower);
////////////////////////FARMING MODE/////////////////
        if(gamepad2.a||farmingMode){
            //loop starter or breaker
            if(gamepad1.dpad_up) {
                OVERIDE = true;
                intaked = false;
                outaked = false;
            }
            else OVERIDE = false;
            if (gamepad1.dpad_up) {
                wristPosition = 0;
                armPosition = 0.5;
                intakePower = 0;
            } else if (gamepad1.dpad_left) {
                armPosition = 0.0;
            }
            if(!OVERIDE) {
                if (armAngle < 20 && gamepad1.a) {
                    hTarget += 50;
                    intakePower = 1;
                }
                if (horizontal_slides.getCurrentPosition() > 25 && !gamepad1.a) {
                    intakePower = 0;
                    armPosition = .6;
                    wristPosition = 1;
                    hTarget = 0;
                    intaked = true;
                }
                if (intaked && getError(horizontal_slides.getCurrentPosition(), hTarget) < 10) {
                    armPosition = 0.9;
                    if (armAngle > 158) intakePower = -1;
                    if (cone_detector.getDistance(DistanceUnit.INCH) < 2) vTarget = HIGH;
                    if (vTarget == HIGH && getError(vPos, vTarget) < 10) {
                        vTarget = 0;
                        intaked = false;
                        outaked = true;
                    }
                }
                if (outaked) {
                    armPosition = 0.01;
                    intakePower = 0;
                    wristPosition = 0;
                    outaked = false;
                }
            }

        }
////////////////////////MANUAL MODE//////////////////
        else if(gamepad2.b){
            horizontal_slides.setPower(-gamepad1.left_stick_x);
            left_vertical_slides.setPower(-gamepad1.right_stick_y);
            right_vertical_slides.setPower(-gamepad1.right_stick_y);
            if(gamepad1.a)intakePower=1;
            else if(gamepad1.b)intakePower=-1;
            else intakePower=0;
            if(gamepad1.dpad_left)wristPosition=0;
            else if(gamepad1.dpad_right)wristPosition=1;
            if(gamepad1.left_bumper&&armCooldown.time()>0.05&&armPosition<1){
                armPosition+=0.025;
                armCooldown.reset();
            }
            else if(gamepad1.left_bumper&&armCooldown.time()>0.05&&armPosition>0){
                armPosition-=0.025;
                armCooldown.reset();
            }
        }
////////////////////////RESET MODE///////////////////
        else if(gamepad2.left_trigger>0&&gamepad2.right_trigger>0&&gamepad1.left_trigger>0&&gamepad1.right_trigger>0){
            left_vertical_slides.setPower(-1);
            right_vertical_slides.setPower(-1);
            horizontal_slides.setPower(-1);
            if(gamepad1.a){
                left_vertical_slides.setMode(STOP_AND_RESET_ENCODER);
                right_vertical_slides.setMode(STOP_AND_RESET_ENCODER);
                horizontal_slides.setMode(STOP_AND_RESET_ENCODER);
                left_vertical_slides.setMode(RUN_WITHOUT_ENCODER);
                right_vertical_slides.setMode(STOP_AND_RESET_ENCODER);
                horizontal_slides.setMode(RUN_WITHOUT_ENCODER);
            }
        }
////////////////////////CIRCUIT MODE/////////////////
        else if(active){
            if(gamepad1.dpad_up) {
                OVERIDE = true;
                intaked = false;
            }
            else OVERIDE = false;
            if (gamepad1.dpad_up) {
                wristPosition = 0;
                armPosition = 0.5;
                intakePower = 0;
            } else if (gamepad1.dpad_left) {
                armPosition = 0.2 ;
                wristPosition = 0;
            }
            if(!OVERIDE) {
                if (armAngle < 40 && gamepad1.a) {
                    hTarget += 50;
                    intakePower = 1;
                    if(horizontal_slides.getCurrentPosition()>350)armPosition=0.01;

                }
                if (horizontal_slides.getCurrentPosition() > 25 && !gamepad1.a) {
                    intakePower = .5;
                    armPosition = .6;
                    wristPosition = 1;
                    hTarget = 0;
                    horizontal_slides.setPower(-.5);
                    intaked = true;
                }
                if (intaked && getError(horizontal_slides.getCurrentPosition(), hTarget) < 25) {
                    armPosition = 0.8;
                    if (armAngle > 145) {
                        sleep(75);
                        intakePower = -.5;
                    }
                    if(cone_detector.getDistance(DistanceUnit.INCH)<2||gamepad1.x) {
                        if (gamepad1.y) vTarget = HIGH;
                        else if (gamepad1.b) vTarget = MID;
                    }
                    if (vTarget > 0 && getError(vPos, vTarget) < 10||(getError(vPos, vTarget) < 50&&gamepad1.y)) {
                        sleep(75);
                        vTarget = 0;
                        armPosition=0.5;
                        wristPosition=0;
                        intakePower=0;
                        intaked = false;
                    }
                }
            }

        }
////////////////////////DRIVE LOGIC//////////////////
        if(gamepad2.right_trigger>0)speedMultiplier=gamepad2.right_trigger;
        else speedMultiplier =1;
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad2.left_stick_y*speedMultiplier,
                        -gamepad2.left_stick_x*speedMultiplier,
                        -gamepad2.right_stick_x*speedMultiplier
                )
        );
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
////////////////////////TELEMETRY////////////////////
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("horizontal position: ",hPos);
        telemetry.addData("horizontal target position: ",hTarget);
        telemetry.addData("horizontal power: ",hPower);
        telemetry.addData("vertical position: ",vPos);
        telemetry.addData("vertical target position: ",vTarget);
        telemetry.addData("vertical power: ",vPower);
        telemetry.addData("arm angle:", armAngle);
        telemetry.addData("loop time: ",loopTime.time());
        loopTime.reset();
        telemetry.update();
    }
    @Override
    public void stop(){

    }
    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }
    public static double verticalPID(double target, PIDController pid,int pos){
        ////////////////////////VERTICAL SLIDES PID//////////
        pid.setPID(vp,vi,vd);
        double vPID = pid.calculate(pos,target);
        double vPower = vPID+vf;
        return vPower;
    }
}
