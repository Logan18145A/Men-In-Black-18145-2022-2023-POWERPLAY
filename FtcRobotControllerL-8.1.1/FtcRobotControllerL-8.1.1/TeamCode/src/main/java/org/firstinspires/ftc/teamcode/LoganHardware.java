package org.firstinspires.ftc.teamcode;
//Import all encoder values
import static org.firstinspires.ftc.teamcode.TeleOP.ArmConstants.LiftConstants;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOP.ArmConstants;
public class LoganHardware {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    //Turret
    public DcMotor Slide = null;
    public Servo GripR = null;
    public Servo GripL = null;
    public DcMotor Turret = null;



}
