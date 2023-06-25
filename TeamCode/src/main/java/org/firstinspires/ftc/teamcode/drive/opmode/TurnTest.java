package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;
    public static double ANGLE = 90; // deg
    private DcMotor ElevatorMotor;
    private CRServo servo;
    private CRServo servo1;
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorMotor= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        servo = hardwareMap.get(CRServo.class, "servo");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(60)
                .addTemporalMarker(.5, () -> {
                    ElevatorMotor.getCurrentPosition();
                    ElevatorMotor.setTargetPosition(ElevatorMotor.getTargetPosition()+2900);
                    ElevatorMotor.setPower(.4);
                })
                .build();

        TrajectorySequence turn = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(165))

                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .forward(14.6)
                .addTemporalMarker(2.5, () -> {
                    ElevatorMotor.getCurrentPosition();
                    ElevatorMotor.setTargetPosition(ElevatorMotor.getTargetPosition()-2200);
                    ElevatorMotor.setPower(.4);
                })
                .addTemporalMarker(2.5, () -> {
                    servo.setPower(1);
                    servo1.setPower(-1);

                })
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                .back(14)
                .build();
        TrajectorySequence turn1 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-325))
                .addTemporalMarker(1, () -> {
                    servo.setPower(1);
                    servo1.setPower(-1);

                })

                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d())
                .forward(40)
                .build();
        drive.followTrajectory(trajectory1);
        drive.followTrajectorySequence(turn);
        drive.followTrajectory(trajectory2);

        drive.followTrajectory(trajectory3);
        drive.followTrajectorySequence(turn1);
        drive.followTrajectory(trajectory4);




        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
