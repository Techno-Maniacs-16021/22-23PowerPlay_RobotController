package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;

@TeleOp(name = "Core (Blocks to Java)")
public class Core extends LinearOpMode {

    private DcMotor core;

    int core_target_pos;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double speed;

        core = hardwareMap.get(DcMotor.class, "core");

        // Put initialization blocks here.
        core.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        core.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        core_target_pos = 0;
        speed = 0.01;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_down) core_target_pos = 1000;
            if (gamepad1.dpad_up) core_target_pos = 0;
            //TODO: IF SLIDES ARE OVERSHOOT DECREASE SPEED
            // Because of the way the control loop is made if you set speed to 0.01 it will travel at 1 power for most of the time
            // Keep speed really low. Try changing it arround 0.01-0.05
            lift(core.getCurrentPosition(),core_target_pos,speed);
        }
    }

    /**
     * Describe this function...
     */
    private void lift(int currentPosition,int target,double speed) {
        double power = 0;
        int error = target-currentPosition;
        if(Math.abs(error)>10) power = error*speed;
        core.setPower(power);
        while (opModeIsActive() && core.isBusy()) {
            idle();
        }
    }
}