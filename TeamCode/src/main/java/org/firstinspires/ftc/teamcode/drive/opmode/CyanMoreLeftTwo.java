package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class CyanMoreLeftTwo extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private DcMotor ElevatorMotor;
    private CRServo servo;
    private CRServo servo1;
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
        ElevatorMotor= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        servo = hardwareMap.get(CRServo.class, "servo");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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





            Trajectory left1 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(46.5,14,Math.toRadians(-24)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })

                    .build();
            Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                    .forward(9)
                    .addTemporalMarker(.3, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })
                    .build();


            Trajectory left2 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-6,5, Math.toRadians(105)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(320);
                        ElevatorMotor.setPower(.7);
                    })




                    .build();
            Trajectory forward1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(23)
                    .addTemporalMarker(1, () -> {
                        servo.setPower(-1);
                        servo1.setPower(1);

                    })
                    .addTemporalMarker(2.4, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })



                    .build();
            Trajectory forward2 = drive.trajectoryBuilder(new Pose2d())
                    .forward(28)
                    .addTemporalMarker(1, () -> {
                        servo.setPower(-1);
                        servo1.setPower(1);

                    })
                    .addTemporalMarker(2.4, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })



                    .build();

            Trajectory left3 = drive.trajectoryBuilder(new Pose2d())
                    .back(23)



                    .build();
            TrajectorySequence leftturn1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-5,-3, Math.toRadians(-100)))

                    .addTemporalMarker(2, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })


                    .build();
            Trajectory left4 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-6,4, Math.toRadians(99)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(180);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();
            Trajectory left5 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-4,3, Math.toRadians(106)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(140);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();
            Trajectory left6 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-5,0, Math.toRadians(101)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(205);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();






            TrajectorySequence leftturn2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-4,-6, Math.toRadians(-105)))

                    .addTemporalMarker(2, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })
                    .build();
            TrajectorySequence parking = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-28,-10, Math.toRadians(30)))
                    .strafeLeft(30)
                    .forward(6)

                    .addTemporalMarker(2, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })
                    .build();












//cone1
            drive.followTrajectory(left1);
            drive.followTrajectory(forward);
//cone2
            drive.followTrajectory(left2);
            drive.followTrajectory(forward1);

            drive.followTrajectory(left3);
            drive.followTrajectorySequence(leftturn1);
            //cone3

            drive.followTrajectory(left6);
            drive.followTrajectory(forward2);

            drive.followTrajectory(left3);
            drive.followTrajectorySequence(leftturn2);
            //parking
//make seperate paths if needed.




            drive.followTrajectorySequence(parking);








            //parking







        }
        else if(tagOfInterest==null || tagOfInterest.id==Middle){
            Trajectory left1 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(46.5,14,Math.toRadians(-24)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })

                    .build();
            Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                    .forward(9)
                    .addTemporalMarker(.3, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })
                    .build();


            Trajectory left2 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-6,5, Math.toRadians(105)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(320);
                        ElevatorMotor.setPower(.7);
                    })




                    .build();
            Trajectory forward1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(23)
                    .addTemporalMarker(1, () -> {
                        servo.setPower(-1);
                        servo1.setPower(1);

                    })
                    .addTemporalMarker(2.4, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })



                    .build();
            Trajectory forward2 = drive.trajectoryBuilder(new Pose2d())
                    .forward(25.5)
                    .addTemporalMarker(1, () -> {
                        servo.setPower(-1);
                        servo1.setPower(1);

                    })
                    .addTemporalMarker(2.4, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })



                    .build();

            Trajectory left3 = drive.trajectoryBuilder(new Pose2d())
                    .back(23)



                    .build();
            TrajectorySequence leftturn1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-5,-3, Math.toRadians(-101)))

                    .addTemporalMarker(2, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })


                    .build();


            Trajectory left6 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-5,0, Math.toRadians(101)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(190);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();






            TrajectorySequence leftturn2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-5,-6, Math.toRadians(-105)))

                    .addTemporalMarker(2, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })
                    .build();

            TrajectorySequence leftturn4 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-10, 2, Math.toRadians(30)))

                    .build();









//cone1
            drive.followTrajectory(left1);
            drive.followTrajectory(forward);
//cone2
            drive.followTrajectory(left2);
            drive.followTrajectory(forward1);

            drive.followTrajectory(left3);
            drive.followTrajectorySequence(leftturn1);
            //cone3

            drive.followTrajectory(left6);
            drive.followTrajectory(forward2);

            drive.followTrajectory(left3);
            drive.followTrajectorySequence(leftturn2);

            //parking
            drive.followTrajectorySequence(leftturn4);


        }
        else if(tagOfInterest==null || tagOfInterest.id==Right){
            Trajectory left1 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(46.5,14,Math.toRadians(-24)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })

                    .build();
            Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                    .forward(9)
                    .addTemporalMarker(.3, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })
                    .build();


            Trajectory left2 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-6,5, Math.toRadians(105)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(320);
                        ElevatorMotor.setPower(.7);
                    })




                    .build();
            Trajectory forward1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(23)
                    .addTemporalMarker(1, () -> {
                        servo.setPower(-1);
                        servo1.setPower(1);

                    })
                    .addTemporalMarker(2.4, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })



                    .build();
            Trajectory forward2 = drive.trajectoryBuilder(new Pose2d())
                    .forward(26)
                    .addTemporalMarker(1, () -> {
                        servo.setPower(-1);
                        servo1.setPower(1);

                    })
                    .addTemporalMarker(2.4, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(2800);
                        ElevatorMotor.setPower(.7);
                    })



                    .build();

            Trajectory left3 = drive.trajectoryBuilder(new Pose2d())
                    .back(23)



                    .build();
            TrajectorySequence leftturn1 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-5,-3, Math.toRadians(-101)))

                    .addTemporalMarker(2, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })


                    .build();
            Trajectory left4 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-6,4, Math.toRadians(99)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(180);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();
            Trajectory left5 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-4,3, Math.toRadians(106)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(140);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();
            Trajectory left6 = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-5,0, Math.toRadians(101)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(177);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();
            Trajectory park = drive.trajectoryBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-8,2, Math.toRadians(45)))
                    .addTemporalMarker(.1, () -> {
                        ElevatorMotor.getCurrentPosition();
                        ElevatorMotor.setTargetPosition(180);
                        ElevatorMotor.setPower(.7);
                    })
                    .build();





            TrajectorySequence leftturn2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-4,-6, Math.toRadians(-105)))

                    .addTemporalMarker(2, () -> {
                        servo.setPower(1);
                        servo1.setPower(-1);

                    })
                    .build();
            TrajectorySequence park6 = drive.trajectorySequenceBuilder(new Pose2d())
                    .turn(Math.toRadians(-68))


                    .build();
            Trajectory park1 = drive.trajectoryBuilder(new Pose2d())
                    .forward(19)
                    .build();
            TrajectorySequence park2 = drive.trajectorySequenceBuilder(new Pose2d())
                    .turn(Math.toRadians(106))


                    .build();
            Trajectory park3 = drive.trajectoryBuilder(new Pose2d())
                    .back(16)
                    .build();











//cone1
            drive.followTrajectory(left1);
            drive.followTrajectory(forward);
//cone2
            drive.followTrajectory(left2);
            drive.followTrajectory(forward1);

            drive.followTrajectory(left3);
            drive.followTrajectorySequence(leftturn1);
            //cone3

            drive.followTrajectory(left6);
            drive.followTrajectory(forward2);

            drive.followTrajectory(left3);
            drive.followTrajectorySequence(leftturn2);
            //parking
//make seperate paths if needed.

            drive.followTrajectorySequence(park6);
            drive.followTrajectory(park1);
            drive.followTrajectorySequence(park2);
            drive.followTrajectory(park3);











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
