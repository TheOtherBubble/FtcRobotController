package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//@Autonomous(name="AutonDrivingDriveOnly", group="AutonTesting")
public class AutonDrivingWIP extends LinearOpMode {

    /* Declare OpMode members. */
    public Hardware robot = new Hardware();
    public ElapsedTime runtime = new ElapsedTime();

    public String xyz = "z";

    static final double     COUNTS_PER_MOTOR_REV = 537.6;    // Currently: Andymark Neverest 20
    static final double     DRIVE_GEAR_REDUCTION = .4;    // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES = 2.95276;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //Gyro
    public BNO055IMU imu;
    public Orientation angles = new Orientation();
    public Acceleration gravity;
    BNO055IMU.Parameters p = new BNO055IMU.Parameters();

    //Vuforia/TF things
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "AYy6NYn/////AAABmTW3q+TyLUMbg/IXWlIG3BkMMq0okH0hLmwj3CxhPhvUlEZHaOAmESqfePJ57KC2g6UdWLN7OYvc8ihGAZSUJ2JPWAsHQGv6GUAj4BlrMCjHvqhY0w3tV/Azw2wlPmls4FcUCRTzidzVEDy+dtxqQ7U5ZtiQhjBZetAcnLsCYb58dgwZEjTx2+36jiqcFYvS+FlNJBpbwmnPUyEEb32YBBZj4ra5jB0v4IW4wYYRKTNijAQKxco33VYSCbH0at99SqhXECURA55dtmmJxYpFlT/sMmj0iblOqoG/auapQmmyEEXt/T8hv9StyirabxhbVVSe7fPsAueiXOWVm0kCPO+KN/TyWYB9Hg/mSfnNu9i9";

    private static final float mmPerInch        = 25.4f;

    // the height of the center of the target image above the floor
    private static final float mmTargetHeight   = (6) * mmPerInch;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public VuforiaTrackables targetsUltimateGoal;
    public List<VuforiaTrackable> allTrackables;

    @Override
    public void runOpMode()
    {
    }

    public void initVuforia()
    {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsUltimateGoal);

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (CAMERA_CHOICE == BACK)
        {
            phoneYRotate = -90;
        }
        else
        {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the came a lens to where it is on the robot.
        // In this example, it is c ntered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


    }

    public void vuforia(List<VuforiaTrackable> allTrackables, VuforiaTrackables targetsUltimateGoal, double timeoutS)
    {
        runtime.reset();
        targetsUltimateGoal.activate();
        while (runtime.seconds() < timeoutS)
        {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables)
            {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible())
                {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null)
                    {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible)
            {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else
                {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    public void initImu()
    {
        p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        p.calibrationDataFile = "BNO055IMUCalibration.json";
        p.loggingEnabled = true;
        p.loggingTag = "IMU";
        p.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(p);
    }

    public void stopAndReset()
    {
        //side motors
        robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lateral motors
        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateAngles()
    {
        try
        {
            angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        catch (NullPointerException e)
        {
            telemetry.addData("Null Pointer Exception", "true");
        }
    }

    public void normalDrive(double lpower, double rpower)
    {

        if (opModeIsActive())
        {
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setPower(lpower);
            robot.fRMotor.setPower(rpower);
            robot.bLMotor.setPower(lpower);
            robot.bRMotor.setPower(rpower);
        }
    }

    public double readAngle(String xyz)
    {
        //Orientation angles;
        Acceleration gravity;
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        updateAngles();
        if (xyz.equals("x"))
        {
            return angles.thirdAngle;
        }
        else if (xyz.equals("y"))
        {
            return angles.secondAngle;
        }
        else if (xyz.equals("z"))
        {
            return angles.firstAngle;
        }
        else
        {
            return 0;
        }
    }

    public void pathComplete(int millisec)
    {
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(millisec);
    }

}