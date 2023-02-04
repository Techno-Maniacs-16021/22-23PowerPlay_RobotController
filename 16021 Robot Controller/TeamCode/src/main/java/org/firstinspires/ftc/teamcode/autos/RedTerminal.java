package org.firstinspires.ftc.teamcode.autos;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class RedTerminal extends LinearOpMode {
    /////////////////////////////////////////////
    ServoImplEx left_arm, right_arm, wrist;
    CRServo left_intake, right_intake;
    DcMotorEx horizontal_slides, vertical_slides;
    AnalogInput rightArmPosition,leftArmPosition;
    /////////////////////////////////////////////
    private PIDController hController,vController;
    public static double hp=0.03,hi=0,hd=0.0005,hTarget = 0;
    public static double vp=0.0225,vi=0,vd=0.0005,vf=0.01,vTarget = 0;
    /////////////////////////////////////////////
    public static double LEFT = 1; //boolean, 0 if false, 1 if true;
    public static double MIDDLE = 0; //boolean, 0 if false, 1 if true;
    public static double RIGHT = 0; //boolean, 0 if false, 1 if true;
    public static double X = 36;
    public static double Y = 5;
    public static double HEAD = 12.5;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ////////////////////////HARDWARE INIT////////////////
                left_arm = hardwareMap.get(ServoImplEx.class, "LA");
        right_arm = hardwareMap.get(ServoImplEx.class, "RA");
        wrist = hardwareMap.get(ServoImplEx.class,"Wrist");
        left_intake = hardwareMap.get(CRServo.class,"LI");
        right_intake = hardwareMap.get(CRServo.class,"RI");
        rightArmPosition = hardwareMap.get(AnalogInput.class,"rArmPos");
        leftArmPosition = hardwareMap.get(AnalogInput.class,"lArmPos");
        vertical_slides = hardwareMap.get(DcMotorEx.class,"V");
        horizontal_slides = hardwareMap.get(DcMotorEx.class,"H");
////////////////////////SET PWM RANGE////////////////
        left_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        right_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        wrist.setPwmRange(new PwmControl.PwmRange(510,2490));
////////////////////////HARDWARE REVERSING///////////
        right_arm.setDirection(ServoImplEx.Direction.REVERSE);
        left_intake.setDirection(CRServo.Direction.REVERSE);
        vertical_slides.setDirection(DcMotorSimple.Direction.REVERSE);
////////////////////////MOTOR BRAKE BEHAVIOR/////////
        vertical_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        horizontal_slides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
////////////////////////ENCODER RESET////////////////
        vertical_slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertical_slides.setMode(RUN_WITHOUT_ENCODER);
        horizontal_slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontal_slides.setMode(RUN_WITHOUT_ENCODER);
////////////////////////INIT POSITIONS///////////////
        left_arm.setPosition(0.5);
        right_arm.setPosition(0.5);
        wrist.setPosition(0);
////////////////////////PID CONTROLLERS//////////////
        hController = new PIDController(hp,hi,hd);
        vController = new PIDController(vp,vi,vd);
////////////////////////DASHBOARD TELEMETRY//////////
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////////////////////////STATUS UPDATE////////////////
        telemetry.addData("Status", "Initialized");
/////////////////////////////////////////////////////
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(36, 64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory auto = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(36, 14))
                .build();
        Trajectory farm = drive.trajectoryBuilder(auto.end())
                .lineToLinearHeading(new Pose2d(36.5,5,Math.toRadians(HEAD)))
                .build();
        Trajectory park = drive.trajectoryBuilder(farm.end())
                .lineTo(new Vector2d(36, 36))
                .build();
        Trajectory left = drive.trajectoryBuilder(park.end())
                .strafeTo(new Vector2d(58, 36))
                .build();
        Trajectory right = drive.trajectoryBuilder(park.end())
                .strafeTo(new Vector2d(12, 36))
                .build();
        while (!isStopRequested()&&!isStarted()){
        }
        waitForStart();

        while (opModeIsActive()) {

            if (LEFT == 1) {
                telemetry.addLine("Parking Left");
                telemetry.update();
            }
            else if (RIGHT == 1) {
                telemetry.addLine("Parking Right");
                telemetry.update();
            }
            else {
                telemetry.addLine("Parking Middle");
                telemetry.update();
            }
            drive.followTrajectory(auto);
            drive.followTrajectory(farm);
            vTarget=3650;
            while (opModeIsActive()&&(Math.abs(vTarget-(vertical_slides.getCurrentPosition()))>20)){
                ////////////////////////VERTICAL SLIDES PID//////////
                vController.setPID(vp,vi,vd);
                int vPos = vertical_slides.getCurrentPosition();
                double vPID = vController.calculate(vPos,vTarget);
                double vPower = vPID+vf;
                vertical_slides.setPower(vPower);
            }
            vTarget=0;
            while (opModeIsActive()&&Math.abs(vTarget-vertical_slides.getCurrentPosition())>10){
                ////////////////////////VERTICAL SLIDES PID//////////
                vController.setPID(vp,vi,vd);
                int vPos = vertical_slides.getCurrentPosition();
                double vPID = vController.calculate(vPos,vTarget);
                double vPower = vPID+vf;
                vertical_slides.setPower(vPower);
            }
            vertical_slides.setPower(0);
            left_arm.setPosition(0.15);
            right_arm.setPosition(0.15);
            hTarget=1950;
            while(opModeIsActive()&&Math.abs(hTarget-horizontal_slides.getCurrentPosition())>10){
                ////////////////////////HORIZONTAL SLIDES PID////////
                hController.setPID(hp,hi,hd);
                int hPos = horizontal_slides.getCurrentPosition();
                double hPID = hController.calculate(hPos,hTarget);
                double hPower = hPID;
                horizontal_slides.setPower(hPower);
            }
            left_intake.setPower(1);
            right_intake.setPower(1);
            sleep(1500);
            left_intake.setPower(0);
            right_intake.setPower(0);
            left_arm.setPosition(.5);
            right_arm.setPosition(.5);
            sleep(500);
            wrist.setPosition(1);
            hTarget=0;
            while(opModeIsActive()&&Math.abs(hTarget-horizontal_slides.getCurrentPosition())>10){
                ////////////////////////HORIZONTAL SLIDES PID////////
                hController.setPID(hp,hi,hd);
                int hPos = horizontal_slides.getCurrentPosition();
                double hPID = hController.calculate(hPos,hTarget);
                double hPower = hPID;
                horizontal_slides.setPower(hPower);
            }
            left_arm.setPosition(.9);
            right_arm.setPosition(.9);
            sleep(500);
            left_intake.setPower(-1);
            right_intake.setPower(-1);
            sleep(500);
            left_intake.setPower(0);
            right_intake.setPower(0);
            vTarget=3650;
            while (opModeIsActive()&&(Math.abs(vTarget-(vertical_slides.getCurrentPosition()))>20)){
                ////////////////////////VERTICAL SLIDES PID//////////
                vController.setPID(vp,vi,vd);
                int vPos = vertical_slides.getCurrentPosition();
                double vPID = vController.calculate(vPos,vTarget);
                double vPower = vPID+vf;
                vertical_slides.setPower(vPower);
            }
            vTarget=0;
            left_arm.setPosition(0);
            right_arm.setPosition(0);
            while (opModeIsActive()&&Math.abs(vTarget-vertical_slides.getCurrentPosition())>10){
                ////////////////////////VERTICAL SLIDES PID//////////
                vController.setPID(vp,vi,vd);
                int vPos = vertical_slides.getCurrentPosition();
                double vPID = vController.calculate(vPos,vTarget);
                double vPower = vPID+vf;
                vertical_slides.setPower(vPower);
            }
            vertical_slides.setPower(0);
            /*drive.followTrajectory(pickup);
            drive.followTrajectory(park);
            if (LEFT == 1) {
                drive.followTrajectory(left);
            }
            else if (RIGHT == 1) {
                drive.followTrajectory(right);
            }*/
            requestOpModeStop();

        }
    }
}