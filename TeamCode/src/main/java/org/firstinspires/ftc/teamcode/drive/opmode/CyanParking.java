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
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class CyanParking extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    public static double DISTANCE = 60; // in
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    AprilTagDetection tagOfInterest;

    int left = 0;
    int Middle = 1;
    int Right = 2;
    /*
    private DcMotor ElevatorMotor;
    private CRServo servo;
    private CRServo servo1;
    //for when elevator is in place
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /*
        //for when elevator is in place (and servos)
        ElevatorMotor= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        servo = hardwareMap.get(CRServo.class, "servo");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */





        while (!isStopRequested() && opModeIsActive()) ;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "OpenCvCamera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(20);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id==Middle||tag.id==Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();

        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }



// free NBA Yungboy

        if(tagOfInterest==null || tagOfInterest.id==left){
            Trajectory trajectory0 = drive.trajectoryBuilder(new Pose2d())
                    .forward(3)
                    .build();
            Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(40)
                    .build();
            TrajectorySequence turn = drive.trajectorySequenceBuilder(new Pose2d())
                    .turn(Math.toRadians(13))

                    .build();
            Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                    .forward(35)
                    .build();
            drive.followTrajectory(trajectory0);
            drive.followTrajectory(trajectory1);
            drive.followTrajectorySequence(turn);


            drive.followTrajectory(trajectory2);
        }
        else if(tagOfInterest==null || tagOfInterest.id==Middle){
            Trajectory trajectory21 = drive.trajectoryBuilder(new Pose2d())
                    .forward(5)
                    .build();
            Trajectory trajectory9 = drive.trajectoryBuilder(new Pose2d())
                    .strafeLeft(8)
                    .build();
            TrajectorySequence turn = drive.trajectorySequenceBuilder(new Pose2d())
                    .turn(Math.toRadians(8))

                    .build();
            Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d())
                    .forward(38)
                    .build();
            drive.followTrajectory(trajectory21);

            drive.followTrajectory(trajectory9);
            drive.followTrajectorySequence(turn);


            drive.followTrajectory(trajectory3);
        }
        else if(tagOfInterest==null || tagOfInterest.id==Right){
            Trajectory trajectory6 = drive.trajectoryBuilder(new Pose2d())
                    .forward(2)
                    .build();
            drive.followTrajectory(trajectory6);
            Trajectory right1 = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(29)
                    .build();
            drive.followTrajectory(right1);
            Trajectory right2 = drive.trajectoryBuilder(new Pose2d())
                    .forward(38)
                    .build();
            drive.followTrajectory(right2);
        }
        waitForStart();

        if (isStopRequested()) return;



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */}


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
