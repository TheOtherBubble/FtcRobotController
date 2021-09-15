package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//public class package org.firstinspires.ftc.teamcode.src;

//import com.qualcomm.robotcore.hardware.DistanceSensor;


public class MecanumHardware
{
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;


    //declaring values for use with encoders
    public String xyz = "z";

    public static final double     COUNTS_PER_MOTOR_REV = 383.6;    // Currently: Andymark Neverest 40
    public static final double     COUNTS_PER_REV_ARM = 1440;
    public static final double     COUNTS_PER_INCH_ARM = COUNTS_PER_REV_ARM/4;
    public static final double     DRIVE_GEAR_REDUCTION = .666;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    public static final double     WHEEL_DIAMETER_INCHES = 3.7795;     // For figuring circumference
    public static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double      ENCODER_COUNTS_PER_REV = 8192;
    public static final double      ENCODER_WHEEL_CIRCUMFERENCE = 4.329;
    public static final double      ENCODER_COUNTS_PER_INCH = ENCODER_COUNTS_PER_REV/ENCODER_WHEEL_CIRCUMFERENCE; //1892.36864358
    /* Local OpMode members. */
    HardwareMap hwMap;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define Motors
        fLMotor = hwMap.get(DcMotor.class, "fLMotor");
        fRMotor = hwMap.get(DcMotor.class, "fRMotor");
        bRMotor = hwMap.get(DcMotor.class, "bRMotor");
        bLMotor = hwMap.get(DcMotor.class, "bLMotor");


        fLMotor.setPower(0);
        bLMotor.setPower(0);
        fRMotor.setPower(0);
        bRMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fLMotor.setDirection(DcMotor.Direction.REVERSE);
        fRMotor.setDirection(DcMotor.Direction.FORWARD);
        bLMotor.setDirection(DcMotor.Direction.REVERSE);
        bRMotor.setDirection(DcMotor.Direction.FORWARD);

    }
}
