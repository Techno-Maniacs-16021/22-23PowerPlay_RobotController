package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Reset_Encoders extends OpMode {
    DcMotorEx horizontal_slides, left_vertical_slides,right_vertical_slides;

    public void init(){
        left_vertical_slides = hardwareMap.get(DcMotorEx.class,"LV");
        right_vertical_slides = hardwareMap.get(DcMotorEx.class,"RV");
        horizontal_slides = hardwareMap.get(DcMotorEx.class,"H");
        left_vertical_slides.setMode(STOP_AND_RESET_ENCODER);
        right_vertical_slides.setMode(STOP_AND_RESET_ENCODER);
        left_vertical_slides.setMode(RUN_WITHOUT_ENCODER);
        right_vertical_slides.setMode(RUN_WITHOUT_ENCODER);
        horizontal_slides.setMode(STOP_AND_RESET_ENCODER);
        horizontal_slides.setMode(RUN_WITHOUT_ENCODER);
    }
    public void loop(){
        left_vertical_slides.setMode(STOP_AND_RESET_ENCODER);
        right_vertical_slides.setMode(STOP_AND_RESET_ENCODER);
        left_vertical_slides.setMode(RUN_WITHOUT_ENCODER);
        right_vertical_slides.setMode(RUN_WITHOUT_ENCODER);
        horizontal_slides.setMode(STOP_AND_RESET_ENCODER);
        horizontal_slides.setMode(RUN_WITHOUT_ENCODER);
    }
}
