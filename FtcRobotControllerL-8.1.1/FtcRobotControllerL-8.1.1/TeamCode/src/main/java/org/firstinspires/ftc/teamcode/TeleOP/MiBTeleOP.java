package org.firstinspires.ftc.teamcode.TeleOP;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*@TeleOp(name = "MiBTeleOPL")
public class MiBTeleOP extends LinearOpMode {
    //drivetrain
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLAsDcMotor = null;
    private DcMotor FRAsDcMotor = null;
    private DcMotor BLAsDcMotor = null;
    private DcMotor BRAsDcMotor = null;
    //Turret
    private DcMotor ArmAsDcMotor = null;
    private DcMotor SlideAsDcMotor = null;
    private Servo GripRAsServo = null;
    private Servo GripLAsServo = null;
    private DcMotor TurretAsDcMotor = null;
    private TouchSensor ArmLimitDownAsTouchSensor = null;
    private LightSensor GripColorAsLightSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        FLAsDcMotor = hardwareMap.get(DcMotor.class, "LF");
        BLAsDcMotor = hardwareMap.get(DcMotor.class, "LB");
        FRAsDcMotor = hardwareMap.get(DcMotor.class, "rightFront");
        BRAsDcMotor = hardwareMap.get(DcMotor.class, "rightBack");

       /* //Lift
        lift = hardwareMap.get(DcMotor.class, "eleMotor");
        Claw = hardwareMap.get(Servo.class, "claw");

        //Set wheels in right direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
*/