package org.firstinspires.ftc.teamcode.autos;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lab.rick.GenericDetector;

@Config
@Autonomous(preselectTeleOp = "Driver_Mode")
public class Blue_Terminal extends LinearOpMode {
    /////////////////////////////////////////////
    ServoImplEx left_arm, right_arm, wrist;
    CRServo left_intake, right_intake;
    DcMotorEx horizontal_slides, vertical_slides;
    AnalogInput rightArmPosition,leftArmPosition;
    RevColorSensorV3 cone_detector;
    /////////////////////////////////////////////
    private PIDController hController,vController;
    public static double hp=0.03,hi=0,hd=0.000,hTarget = 0;
    public static double  vp=0.02,vi=0,vd=0.0003,vf=0.01,vTarget = 0;
    /////////////////////////////////////////////
    public static double REPEAT = 5; //number of extra cones;
    public static double X = -39;
    public static double Y = 7;
    public static double HEAD = 168;
    public static double tangent = 160;
    public static double HTARGET = 2100;
    public static double VTARGET = 3495;
    public static double HBUFFER = 500;
    public static double topConeAngle = .16;
    boolean gate = false;
    private GenericDetector rf = null;
    private String result = "";
    ElapsedTime intake = new ElapsedTime();
    ElapsedTime elapsedTime = new ElapsedTime();
    ElapsedTime time = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();
    double targetArmPos = topConeAngle;

    @Override
    public void runOpMode() {
        vp=0.02;vi=0;vd=0.0003;vf=0.01;vTarget = 0;hp=0.03;hi=0;hd=0.000;hTarget = 0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ////////////////////////HARDWARE INIT////////////////
        left_arm = hardwareMap.get(ServoImplEx.class, "LA");
        right_arm = hardwareMap.get(ServoImplEx.class, "RA");
        wrist = hardwareMap.get(ServoImplEx.class,"Wrist");
        left_intake = hardwareMap.get(CRServo.class,"LI");
        right_intake = hardwareMap.get(CRServo.class,"RI");
        rightArmPosition = hardwareMap.get(AnalogInput.class,"rArmPos");
        leftArmPosition = hardwareMap.get(AnalogInput.class,"lArmPos");
        cone_detector = hardwareMap.get(RevColorSensorV3.class,"coneDetector");
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


        Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        Trajectory farm = drive.trajectoryBuilder(startPose,true)
                .splineToSplineHeading(new Pose2d(X,Y,Math.toRadians(HEAD)),tangent)
                .build();
        Trajectory park = drive.trajectoryBuilder(farm.end())
                .lineToLinearHeading(new Pose2d(-36, 12,Math.toRadians(180)))
                .build();
        Trajectory right = drive.trajectoryBuilder(farm.end())
                .lineToLinearHeading(new Pose2d(-58, 12,Math.toRadians(180)))
                .build();
        Trajectory left = drive.trajectoryBuilder(farm.end())
                .lineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(90)))
                .build();

        try {
            //initialize the bot

            //initialize the detector. It will run on its own thread continuously
            rf = new GenericDetector(this.hardwareMap,  this,  telemetry);
            Thread detectThread = new Thread(rf);
            detectThread.start();
            telemetry.update();
        } catch (Exception ex) {
            telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
            telemetry.update();
            sleep(5000);
            return;
        }
        waitForStart();
        elapsedTime.reset();
        time.reset();
        rf.stopDetection();

        result = rf.getResult();

        while (opModeIsActive()) {

            if (result.equals("LEFT")) {
                telemetry.addLine("Parking Left");
                telemetry.update();
            }
            else if (result.equals("RIGHT")) {
                telemetry.addLine("Parking Right");
                telemetry.update();
            }
            else {
                telemetry.addLine("Parking Middle");
                telemetry.update();
            }
            //drive.followTrajectory(auto);
            drive.followTrajectory(farm);
            //Cone_Farm//
            for (int i=0;i<REPEAT;i++) {
                vTarget = VTARGET;
                hTarget = HTARGET - HBUFFER;
                left_arm.setPosition(targetArmPos);
                right_arm.setPosition(targetArmPos);
                wrist.setPosition(0);
                while (opModeIsActive() && getError(vertical_slides.getCurrentPosition(), vTarget) > 30) {
                    vertical_slides.setPower(verticalPID(vTarget, vController, vertical_slides.getCurrentPosition()));
                    horizontal_slides.setPower(horizontalPID(hTarget, hController, horizontal_slides.getCurrentPosition()));
                    telemetry.addData("loop time: ", loopTime.time());
                    loopTime.reset();
                    telemetry.update();
                }
                left_intake.setPower(1);
                right_intake.setPower(1);
                vTarget = 0;
                hTarget = HTARGET;
                while ((opModeIsActive() && getError(vertical_slides.getCurrentPosition(), vTarget) > 20)||(opModeIsActive()&&getError(horizontal_slides.getCurrentPosition(), hTarget) > 30)) {

                    vertical_slides.setPower(verticalPID(vTarget, vController, vertical_slides.getCurrentPosition()));
                    if(vertical_slides.getCurrentPosition()<2500) horizontal_slides.setPower(horizontalPID(hTarget, hController, horizontal_slides.getCurrentPosition()));
                    if (getError(horizontal_slides.getCurrentPosition(), hTarget) > 60) {
                        gate = true;
                        intake.reset();
                    }
                    if (getError(horizontal_slides.getCurrentPosition(), hTarget)<60&&gate&intake.time()>.15) {
                        left_intake.setPower(0.01);
                        right_intake.setPower(0.01);
                        left_arm.setPosition(.5);
                        right_arm.setPosition(.5);
                    }
                    if (getError(horizontal_slides.getCurrentPosition(), hTarget)<60&&gate&&intake.time()>.3) {
                        gate=false;
                        wrist.setPosition(1);
                        hTarget=0;
                    }
                    telemetry.addData("loop time: ", loopTime.time());
                    loopTime.reset();
                    telemetry.update();

                }
                vertical_slides.setPower(-.5);
                while (opModeIsActive() && getError(horizontal_slides.getCurrentPosition(), hTarget) > 10) horizontal_slides.setPower(horizontalPID(hTarget, hController, horizontal_slides.getCurrentPosition()));
                horizontal_slides.setPower(-.5);
                left_arm.setPosition(.9);
                right_arm.setPosition(.9);
                sleep(350);
                left_intake.setPower(-1);
                right_intake.setPower(-1);
                time.reset();
                while (opModeIsActive()&&cone_detector.getDistance(DistanceUnit.INCH)>1&&time.time()<.5){/**COOLDOWN FOR CONE**/}
                left_intake.setPower(0);
                right_intake.setPower(0);
                left_arm.setPosition(.5);
                right_arm.setPosition(.5);
                targetArmPos -= (topConeAngle/5);
                sleep(50);
                //Cone_Farm//
            }
            vTarget = VTARGET;
            while (opModeIsActive() && getError(vertical_slides.getCurrentPosition(), vTarget) > 30) vertical_slides.setPower(verticalPID(vTarget,vController,vertical_slides.getCurrentPosition()));
            vTarget = 0;
            vertical_slides.setPower(-1);
            sleep(150);
            if (result.equals("LEFT")) {
                drive.followTrajectory(left);
            }
            else if (result.equals("RIGHT")) {
                drive.followTrajectory(right);
            }
            else {
                drive.followTrajectory(park);
            }
            while (opModeIsActive() && getError(vertical_slides.getCurrentPosition(), vTarget) > 20) vertical_slides.setPower(verticalPID(vTarget,vController,vertical_slides.getCurrentPosition()));
            requestOpModeStop();

        }
    }
    public static double verticalPID(double target, PIDController pid,int pos){
        ////////////////////////VERTICAL SLIDES PID//////////
        pid.setPID(vp,vi,vd);
        double vPID = pid.calculate(pos,target);
        double vPower = vPID+vf;
        return vPower;
    }
    public static double horizontalPID(double target,PIDController pid, int pos){
        ////////////////////////HORIZONTAL SLIDES PID////////
        pid.setPID(hp,hi,hd);
        double hPID = pid.calculate(pos,target);
        double hPower = hPID;
        return hPower;
    }
    public static int getError(int current, double target){
        int error = Math.abs((int)target-current);
        return error;
    }
}
