package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//public class package org.firstinspires.ftc.teamcode.src;

//import com.qualcomm.robotcore.hardware.DistanceSensor;


public class Hardware
{
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;

    public DcMotor launcherMotor;

    public Servo launcherServo;

    public DcMotor intakeMotor;

    //declaring values for use with encoders
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.19685039;     // For fwiguring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define Motors
        fLMotor = hwMap.get(DcMotor.class, "fLMotor");
        fRMotor = hwMap.get(DcMotor.class, "fRMotor");
        bRMotor = hwMap.get(DcMotor.class, "bRMotor");
        bLMotor = hwMap.get(DcMotor.class, "bLMotor");

        launcherMotor = hwMap.get(DcMotor.class, "launcherMotor");

        launcherServo = hwMap.get(Servo.class, "launcherServo");

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        fLMotor.setPower(0);
        bLMotor.setPower(0);
        fRMotor.setPower(0);
        bRMotor.setPower(0);
        intakeMotor.setPower(0);

        launcherMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        fLMotor.setDirection(DcMotor.Direction.REVERSE);
        fRMotor.setDirection(DcMotor.Direction.FORWARD);
        bLMotor.setDirection(DcMotor.Direction.REVERSE);
        bRMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        launcherServo.scaleRange(0, 0.3);

        launcherServo.setPosition(0);
    }
}
