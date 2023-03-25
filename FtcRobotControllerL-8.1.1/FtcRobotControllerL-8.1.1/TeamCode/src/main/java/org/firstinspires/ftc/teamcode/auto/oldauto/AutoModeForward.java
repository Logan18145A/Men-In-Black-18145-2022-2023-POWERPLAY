package org.firstinspires.ftc.teamcode.auto.oldauto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoForward")
public class AutoModeForward extends LinearOpMode {
    //private means setting value to nothing, only when setting to null
    private ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront= null;
    public DcMotor rightBack  = null;

   /* public DcMotor lift = null;
    public Servo Claw = null; */



    public final static double speed = 0.6;
    public final static long time = 1800;
    public void left(double power,long time) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
        sleep(time);
    }

    public void right(double power,long time) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
        sleep(time);
    }

    public void stopMove() {
        forward(0,0);
    }

    public void forward(double power,long time) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        sleep(time);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

       /* lift = hardwareMap.get(DcMotor.class, "eleMotor");
        Claw = hardwareMap.get(Servo.class, "claw"); */

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        //forward(speed,1800);
        TrajectorySequence MiBLEFTMIDDLE = drive.trajectorySequenceBuilder(new Pose2d(35.37, -64.58, Math.toRadians(90.57)))
                .splineTo(new Vector2d(28.10, -33.00), Math.toRadians(140.53))
                .splineTo(new Vector2d(9.27, -35.52), Math.toRadians(128.87))
                .splineTo(new Vector2d(10.01, -12.23), Math.toRadians(88.18))
                .splineTo(new Vector2d(35.96, -10.31), Math.toRadians(4.25))
                .splineTo(new Vector2d(60.43, -11.64), Math.toRadians(-3.12))
                .splineTo(new Vector2d(24.40, -10.46), Math.toRadians(178.11))
                .splineTo(new Vector2d(26.47, -10.75), Math.toRadians(-8.13))
                .splineTo(new Vector2d(36.70, -12.23), Math.toRadians(-8.25))
                .build();

        stopMove();

    }

}
