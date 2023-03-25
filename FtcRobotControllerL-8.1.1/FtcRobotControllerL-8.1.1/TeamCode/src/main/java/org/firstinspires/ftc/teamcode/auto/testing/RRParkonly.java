package org.firstinspires.ftc.teamcode.auto.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LoganHardware;
import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.OpenCV.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "ParkOnly")
public class RRParkonly extends LinearOpMode{
     /*Pose2d startPose           = new Pose2d(-37, 63, Math.toRadians(90));
    //Based off the direction you setup
    Pose2d RightBlueStack      = new Pose2d(-60, 12, Math.toRadians(180));
    Pose2d LeftBlueStack       = new Pose2d(60, 12, Math.toRadians(180));
    Pose2d RightRedStack       = new Pose2d(60, -12, Math.toRadians(0));
    Pose2d LeftRedStack        = new Pose2d(-60, -12, Math.toRadians(180));
    //Vectors for Consistency
    Vector2d RightBlueStackV      = new Vector2d(-60,12);
    Vector2d LeftBlueStackV       = new Vector2d(60,12);
    Vector2d RightRedStackV       = new Vector2d(60,-12);
    Vector2d LeftRedStackV        = new Vector2d(-60,-12);

    //Poles
    Pose2d PreLoadMiddleHigh   = new Pose2d(10.5,   -23,Math.toRadians(90));
    Pose2d RightHighPole       = new Pose2d(24,-12,Math.toRadians(180)); */
    //Parking
    Vector2d ParkingLeft = new Vector2d(60,-34);
    Vector2d ParkingRight = new Vector2d(10,-34);
    //AprilTags
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of Sleeve
    int Left = 1;
    int Middle = 2;
    int Right = 3;

    AprilTagDetection tagOfInterest = null;

    static final double FEET_PER_METER = 3.28084;
    @Override
    public void runOpMode() throws InterruptedException {
        LoganHardware hardware = new LoganHardware();

        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
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
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Drive Pose
        Pose2d startPose = new Pose2d(34.97, -60.52, Math.toRadians(90.00));

        drive.setPoseEstimate(startPose);
       Trajectory Forward = drive.trajectoryBuilder(startPose)
               .forward(27)
               .build();
        Trajectory parkspot1 = drive.trajectoryBuilder(Forward.end())
                .lineTo(ParkingLeft)
                .build();
        // Unneeded Trajectory parkspot2 = drive.trajectoryBuilder()

        Trajectory parkspot3 = drive.trajectoryBuilder(Forward.end())
                .lineTo(ParkingRight)
                .build();

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

        telemetry.setMsTransmissionInterval(50);

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
                    if(tag.id == Left ||tag.id == Middle ||tag.id == Right)
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
            sleep(20);
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
        drive.followTrajectory(Forward);


        /* Input Parking Code*/
        if(tagOfInterest == null)
        {
        drive.followTrajectory(Forward);
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if(tagOfInterest.id == Left)
            {
            drive.followTrajectory(parkspot1);
            }
            else if((tagOfInterest == null) || (tagOfInterest.id == Middle))
            {
            drive.followTrajectory(Forward);
            }
            else if(tagOfInterest.id == Right)
            {
            drive.followTrajectory(parkspot3);
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}

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

