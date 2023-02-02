package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class RedTerminal extends LinearOpMode {

    public static double LEFT = 1; //boolean, 0 if false, 1 if true;
    public static double MIDDLE = 0; //boolean, 0 if false, 1 if true;
    public static double RIGHT = 0; //boolean, 0 if false, 1 if true;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPose = new Pose2d(36, 64, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory clearCone = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(36, 5))
                .build();
        Trajectory auto = drive.trajectoryBuilder(clearCone.end())
                .lineTo(new Vector2d(36, 12))
                .build();
        Trajectory pickup = drive.trajectoryBuilder(auto.end())
                .lineToLinearHeading(new Pose2d(36,14,0))
                .build();
        Trajectory park = drive.trajectoryBuilder(pickup.end())
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
            drive.followTrajectory(clearCone);
            drive.followTrajectory(auto);
            drive.followTrajectory(pickup);
            drive.followTrajectory(park);
            if (LEFT == 1) {
                drive.followTrajectory(left);
            }
            else if (RIGHT == 1) {
                drive.followTrajectory(right);
            }
            requestOpModeStop();

        }
    }
}