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

package org.firstinspires.ftc.teamcode.lab.rick;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.RobotIntialization;
import org.firstinspires.ftc.teamcode.tfrec.Detector;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@TeleOp
public class testDriverMode extends OpMode
{
    /////////////////////////////////////////////
    ServoImplEx left_arm, right_arm, wrist;
    CRServo left_intake, right_intake;
    DcMotorEx horizontal_slides, vertical_slides;
    AnalogInput rightArmPosition,leftArmPosition;
    /////////////////////////////////////////////
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime wristCooldown = new ElapsedTime();
    private ElapsedTime intakeMacroCooldown = new ElapsedTime();
    private ElapsedTime outTakeCooldown = new ElapsedTime();
    private ElapsedTime intakeSlideCooldown = new ElapsedTime();
    private SampleMecanumDrive drive;
    private RobotIntialization robotIntialization;
    private PIDController hController,vController;
    public static double hp=0.03,hi=0,hd=0.0005,hTarget = 0;
    public static double vp=0.025,vi=0,vd=0.0005,vf=0.02,vTarget = 0;
    int gamepad_A_Release = 0;
    boolean aRealeased = false;
    /////////////////////////////////////////////
    @Override
    public void init() {
////////////////////////HARDWARE INIT////////////////
        drive = new SampleMecanumDrive(hardwareMap);
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
        left_arm.setPosition(0);
        right_arm.setPosition(0);
        wrist.setPosition(0);
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
    }
    @Override
    public void init_loop() {

    }
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
////////////////////////HORIZONTAL SLIDES PID////////
        hController.setPID(hp,hi,hd);
        int hPos = horizontal_slides.getCurrentPosition();
        double hPID = hController.calculate(hPos,hTarget);
        double hPower = hPID;
        horizontal_slides.setPower(hPower);
        //horizontal_slides.setPower(gamepad1.left_stick_x);
////////////////////////VERTICAL SLIDES PID//////////
        vController.setPID(vp,vi,vd);
        int vPos = vertical_slides.getCurrentPosition();
        double vPID = vController.calculate(vPos,vTarget);
        double vPower = vPID+vf;
        vertical_slides.setPower(vPower);
        //vertical_slides.setPower(gamepad1.left_sti ck_y);
////////////////////////ARM PID VARS/////////////////
        double leftArmAngle = ((leftArmPosition.getVoltage()-0.82) /3.3) * 360;
        double rightArmAngle = ((2.499-rightArmPosition.getVoltage())/3.3) * 360;
        double armAngle = (leftArmAngle+rightArmAngle)/2;
////////////////////////ARM LOGIC////////////////////
        if(gamepad1.dpad_left){
            left_arm.setPosition(0);
            right_arm.setPosition(0);
            wrist.setPosition(0);
            hTarget=750;
            vTarget=0;
            intakeMacroCooldown.reset();
        }
        else if(gamepad1.dpad_up){
            left_arm.setPosition(0.5);
            right_arm.setPosition(0.5);
            wrist.setPosition(1);
            intakeMacroCooldown.reset();
        }
        else if(gamepad1.dpad_right){
            left_arm.setPosition(0.55);
            right_arm.setPosition(0.55);
            wrist.setPosition(1);
            intakeMacroCooldown.reset();
        }
        /*else if(armAngle>100&&intakeMacroCooldown.time()>0.75){
            left_arm.setPosition(0.85);
            right_arm.setPosition(0.85);
            wrist.setPosition(1);

            if(armAngle>152&&intakeMacroCooldown.time()>0.8) {
                left_intake.setPower(-1);
                right_intake.setPower(-1);
            }
        }*/
////////////////////////INTAKE LOGIC/////////////////
        if(gamepad_A_Release!=0&&!gamepad1.a){
            gamepad_A_Release=0;
            aRealeased=true;
        }
        if(aRealeased){
            hTarget=0;
            left_arm.setPosition(0.55);
            right_arm.setPosition(0.55);
            wrist.setPosition(1);
            if(Math.abs(hTarget-hPos)<10) {
                left_arm.setPosition(0.9);
                right_arm.setPosition(0.9);
                intakeMacroCooldown.reset();
                aRealeased=false;
            }
        }
        if(gamepad1.a){
            left_intake.setPower(1);
            right_intake.setPower(1);
            hTarget+=10;
            gamepad_A_Release++;
        }
        else if(armAngle>152&&intakeMacroCooldown.time()>0.8) {
            left_intake.setPower(-1);
            right_intake.setPower(-1);
            if(intakeMacroCooldown.time()>1.5)vTarget=3500;
        }
        else if(gamepad1.b){
            left_intake.setPower(-1);
            right_intake.setPower(-1);
        }
        else{
            left_intake.setPower(0.01);
            right_intake.setPower(0.01);
        }
////////////////////////DRIVE LOGIC//////////////////
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad2.left_stick_y,
                        -gamepad2.left_stick_x,
                        -gamepad2.right_stick_x
                )
        );
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
////////////////////////TELEMETRY////////////////////
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("leftArmAngle: ", leftArmAngle);
        telemetry.addData("rightArmAngle: ", rightArmAngle);
        telemetry.addData("leftArmVoltage: ", leftArmPosition.getVoltage()-0.82);
        telemetry.addData("rightArmVoltage: ", 2.499-rightArmPosition.getVoltage());
        telemetry.addData("horizontal position: ",hPos);
        telemetry.addData("horizontal target position: ",hTarget);
        telemetry.addData("horizontal power: ",hPower);
        telemetry.addData("vertical position: ",vPos);
        telemetry.addData("vertical target position: ",vTarget);
        telemetry.addData("vertical power: ",vPower);
        telemetry.update();
/////////////////////////////////////////////////////
    }
    @Override
    public void stop() {

    }
}