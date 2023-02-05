package org.firstinspires.ftc.teamcode.modules;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lab.rick.GenericDetector;
@Config
public class Robot {
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
    public static double REPEAT = 5; //number of extra cones;
    //public static double X = 36;
    //public static double Y = 5;
    //public static double HEAD = 12.5;
    public static double HTARGET = 2000;
    public static double VTARGET = 3500;
    public static double HBUFFER = 500;
    boolean gate = false;
    private GenericDetector rf = null;
    private String result = "";
    ElapsedTime intake = new ElapsedTime();
    double targetArmPos = 0.2;

    public Robot(HardwareMap hardwareMap){
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
    }

    public ServoImplEx getLeft_arm() {
        return left_arm;
    }

    public void setLeft_arm(ServoImplEx left_arm) {
        this.left_arm = left_arm;
    }

    public ServoImplEx getRight_arm() {
        return right_arm;
    }

    public void setRight_arm(ServoImplEx right_arm) {
        this.right_arm = right_arm;
    }

    public ServoImplEx getWrist() {
        return wrist;
    }

    public void setWrist(ServoImplEx wrist) {
        this.wrist = wrist;
    }

    public CRServo getLeft_intake() {
        return left_intake;
    }

    public void setLeft_intake(CRServo left_intake) {
        this.left_intake = left_intake;
    }

    public CRServo getRight_intake() {
        return right_intake;
    }

    public void setRight_intake(CRServo right_intake) {
        this.right_intake = right_intake;
    }

    public DcMotorEx getHorizontal_slides() {
        return horizontal_slides;
    }

    public void setHorizontal_slides(DcMotorEx horizontal_slides) {
        this.horizontal_slides = horizontal_slides;
    }

    public DcMotorEx getVertical_slides() {
        return vertical_slides;
    }

    public void setVertical_slides(DcMotorEx vertical_slides) {
        this.vertical_slides = vertical_slides;
    }

    public AnalogInput getRightArmPosition() {
        return rightArmPosition;
    }

    public void setRightArmPosition(AnalogInput rightArmPosition) {
        this.rightArmPosition = rightArmPosition;
    }

    public AnalogInput getLeftArmPosition() {
        return leftArmPosition;
    }

    public void setLeftArmPosition(AnalogInput leftArmPosition) {
        this.leftArmPosition = leftArmPosition;
    }

    public PIDController gethController() {
        return hController;
    }

    public void sethController(PIDController hController) {
        this.hController = hController;
    }

    public PIDController getvController() {
        return vController;
    }

    public void setvController(PIDController vController) {
        this.vController = vController;
    }

    public static double getHp() {
        return hp;
    }

    public static void setHp(double hp) {
        Robot.hp = hp;
    }

    public static double getHi() {
        return hi;
    }

    public static void setHi(double hi) {
        Robot.hi = hi;
    }

    public static double getHd() {
        return hd;
    }

    public static void setHd(double hd) {
        Robot.hd = hd;
    }

    public static double gethTarget() {
        return hTarget;
    }

    public static void sethTarget(double hTarget) {
        Robot.hTarget = hTarget;
    }

    public static double getVp() {
        return vp;
    }

    public static void setVp(double vp) {
        Robot.vp = vp;
    }

    public static double getVi() {
        return vi;
    }

    public static void setVi(double vi) {
        Robot.vi = vi;
    }

    public static double getVd() {
        return vd;
    }

    public static void setVd(double vd) {
        Robot.vd = vd;
    }

    public static double getVf() {
        return vf;
    }

    public static void setVf(double vf) {
        Robot.vf = vf;
    }

    public static double getvTarget() {
        return vTarget;
    }

    public static void setvTarget(double vTarget) {
        Robot.vTarget = vTarget;
    }

    public static double getREPEAT() {
        return REPEAT;
    }

    public static void setREPEAT(double REPEAT) {
        Robot.REPEAT = REPEAT;
    }

    public static double getHTARGET() {
        return HTARGET;
    }

    public static void setHTARGET(double HTARGET) {
        Robot.HTARGET = HTARGET;
    }

    public static double getVTARGET() {
        return VTARGET;
    }

    public static void setVTARGET(double VTARGET) {
        Robot.VTARGET = VTARGET;
    }

    public static double getHBUFFER() {
        return HBUFFER;
    }

    public static void setHBUFFER(double HBUFFER) {
        Robot.HBUFFER = HBUFFER;
    }

    public boolean isGate() {
        return gate;
    }

    public void setGate(boolean gate) {
        this.gate = gate;
    }

    public GenericDetector getRf() {
        return rf;
    }

    public void setRf(GenericDetector rf) {
        this.rf = rf;
    }

    public String getResult() {
        return result;
    }

    public void setResult(String result) {
        this.result = result;
    }

    public ElapsedTime getIntake() {
        return intake;
    }

    public void setIntake(ElapsedTime intake) {
        this.intake = intake;
    }

    public double getTargetArmPos() {
        return targetArmPos;
    }

    public void setTargetArmPos(double targetArmPos) {
        this.targetArmPos = targetArmPos;
    }
}
