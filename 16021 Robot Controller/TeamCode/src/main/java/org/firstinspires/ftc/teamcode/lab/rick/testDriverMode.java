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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    DcMotor horizontal_slides, vertical_slides;
    AnalogInput rightArmPosition,leftArmPosition;
    /////////////////////////////////////////////
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime wristCooldown = new ElapsedTime();
    private ElapsedTime intakeMacroCooldown = new ElapsedTime();
    double wristPos = 0 ;
    /////////////////////////////////////////////
    @Override
    public void init() {
        /////////////////////////////////////////////
        left_arm = hardwareMap.get(ServoImplEx.class, "LA");
        right_arm = hardwareMap.get(ServoImplEx.class, "RA");
        wrist = hardwareMap.get(ServoImplEx.class,"Wrist");
        left_intake = hardwareMap.get(CRServo.class,"LI");
        right_intake = hardwareMap.get(CRServo.class,"RI");
        rightArmPosition = hardwareMap.get(AnalogInput.class,"rArmPos");
        leftArmPosition = hardwareMap.get(AnalogInput.class,"lArmPos");
        vertical_slides = hardwareMap.get(DcMotor.class,"V");
        horizontal_slides = hardwareMap.get(DcMotor.class,"H");
        /////////////////////////////////////////////
        left_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        right_arm.setPwmRange(new PwmControl.PwmRange(500,2500));
        wrist.setPwmRange(new PwmControl.PwmRange(510,2490));
        /////////////////////////////////////////////
        right_arm.setDirection(ServoImplEx.Direction.REVERSE);
        left_intake.setDirection(CRServo.Direction.REVERSE);
        horizontal_slides.setDirection(DcMotorSimple.Direction.REVERSE);
        /////////////////////////////////////////////
        vertical_slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontal_slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /////////////////////////////////////////////
        left_arm.setPosition(0);
        right_arm.setPosition(0);
        wrist.setPosition(0);
        /////////////////////////////////////////////
        telemetry.addData("Status", "Initialized");
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
        double leftArmAngle = ((leftArmPosition.getVoltage()-0.82) /3.3) * 360;
        double rightArmAngle = ((2.499-rightArmPosition.getVoltage())/3.3) * 360;
        double armAngle = (leftArmAngle+rightArmAngle)/2;
        vertical_slides.setPower(gamepad1.left_stick_y);
        horizontal_slides.setPower(gamepad1.left_stick_x);
        if(gamepad1.dpad_left){
            left_arm.setPosition(0);
            right_arm.setPosition(0);
            wristPos = 0;
            intakeMacroCooldown.reset();
        }
        else if(gamepad1.dpad_up){
            left_arm.setPosition(0.5);
            right_arm.setPosition(0.5);
            wristPos = .5;
            intakeMacroCooldown.reset();
        }
        else if(gamepad1.dpad_right){
            left_arm.setPosition(0.55);
            right_arm.setPosition(0.55);
            wristPos = 1;
            intakeMacroCooldown.reset();
        }
        else if(armAngle>100&&intakeMacroCooldown.time()>0.75){
            left_arm.setPosition(0.85);
            right_arm.setPosition(0.85);
            wristPos = 1;

            if(armAngle>152&&intakeMacroCooldown.time()>0.8) {
                left_intake.setPower(-1);
                right_intake.setPower(-1);
            }
        }
        if(gamepad1.a){
            left_intake.setPower(1);
            right_intake.setPower(1);
        }
        else if(gamepad1.b){
            left_intake.setPower(-1);
            right_intake.setPower(-1);
        }
        else{
            left_intake.setPower(0.01);
            right_intake.setPower(0.01);
        }
        if(gamepad1.right_trigger>0&&wristCooldown.time()>0.1){
            wristPos+=0.01;
            wristCooldown.reset();
        }
        else if(gamepad1.left_trigger>0&&wristCooldown.time()>0.1){
            wristPos-=0.01;
            wristCooldown.reset();
        }
        wrist.setPosition(wristPos);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("WristPosition: ",wristPos);
        telemetry.addData("leftArmAngle: ", leftArmAngle);
        telemetry.addData("rightArmAngle: ", rightArmAngle);
        telemetry.addData("leftArmVoltage: ", leftArmPosition.getVoltage()-0.82);
        telemetry.addData("rightArmVoltage: ", 2.499-rightArmPosition.getVoltage());
        telemetry.update();
    }
    @Override
    public void stop() {

    }
}
