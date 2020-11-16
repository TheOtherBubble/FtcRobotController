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
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//@Autonomous(name="AutonDrivingDriveOnly", group="AutonTesting")
public class AutonDrivingWIP extends LinearOpMode {

    //
    //OPMODE MEMBERS
    public Hardware robot = new Hardware();
    public ElapsedTime runtime = new ElapsedTime();



    //
    //MEASURING CONSTANTS
    //
    static final double     COUNTS_PER_MOTOR_REV = 537.6;    // Currently: Andymark Neverest 20
    static final double     DRIVE_GEAR_REDUCTION = .4;    // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES = 2.95276;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable



    //
    //VUFORIA
    //
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

    public VuforiaTrackables targetsUltimateGoal = new VuforiaTrackables() {
        @Override public void setName(String name) { }
        @Override public String getName() { return null; }
        @Override public void activate() { }
        @Override public void deactivate() { }
        @Override public VuforiaLocalizer getLocalizer() { return null; }
        @Override public int size() { return 0; }
        @Override public boolean isEmpty() { return false; }
        @Override public boolean contains(Object o) { return false; }
        @Override public Iterator<VuforiaTrackable> iterator() { return null; }
        @Override public Object[] toArray() { return new Object[0]; }
        @Override public <T> T[] toArray(T[] a) { return null; }
        @Override public boolean add(VuforiaTrackable vuforiaTrackable) { return false; }
        @Override public boolean remove(Object o) { return false; }
        @Override public boolean containsAll(Collection<?> c) { return false; }
        @Override public boolean addAll(Collection<? extends VuforiaTrackable> c) { return false; }
        @Override public boolean addAll(int index, Collection<? extends VuforiaTrackable> c) { return false; }
        @Override public boolean removeAll(Collection<?> c) { return false; }
        @Override public boolean retainAll(Collection<?> c) { return false; }
        @Override public void clear() { }
        @Override public VuforiaTrackable get(int index) { return null; }
        @Override public VuforiaTrackable set(int index, VuforiaTrackable element) { return null; }
        @Override public void add(int index, VuforiaTrackable element) { }
        @Override public VuforiaTrackable remove(int index) { return null; }
        @Override public int indexOf(Object o) { return 0; }
        @Override public int lastIndexOf(Object o) { return 0; }
        @Override public ListIterator<VuforiaTrackable> listIterator() { return null; }
        @Override public ListIterator<VuforiaTrackable> listIterator(int index) { return null; }
        @Override public List<VuforiaTrackable> subList(int fromIndex, int toIndex) { return null; }
    };
    public List<VuforiaTrackable> allTrackables = new List<VuforiaTrackable>() {
        @Override
        public int size() {
            return 0;
        }

        @Override
        public boolean isEmpty() {
            return false;
        }

        @Override
        public boolean contains(Object o) {
            return false;
        }

        @Override
        public Iterator<VuforiaTrackable> iterator() {
            return null;
        }

        @Override
        public Object[] toArray() {
            return new Object[0];
        }

        @Override
        public <T> T[] toArray(T[] a) {
            return null;
        }

        @Override
        public boolean add(VuforiaTrackable vuforiaTrackable) {
            return false;
        }

        @Override
        public boolean remove(Object o) {
            return false;
        }

        @Override
        public boolean containsAll(Collection<?> c) {
            return false;
        }

        @Override
        public boolean addAll(Collection<? extends VuforiaTrackable> c) {
            return false;
        }

        @Override
        public boolean addAll(int index, Collection<? extends VuforiaTrackable> c) {
            return false;
        }

        @Override
        public boolean removeAll(Collection<?> c) {
            return false;
        }

        @Override
        public boolean retainAll(Collection<?> c) {
            return false;
        }

        @Override
        public void clear() {

        }

        @Override
        public VuforiaTrackable get(int index) {
            return null;
        }

        @Override
        public VuforiaTrackable set(int index, VuforiaTrackable element) {
            return null;
        }

        @Override
        public void add(int index, VuforiaTrackable element) {

        }

        @Override
        public VuforiaTrackable remove(int index) {
            return null;
        }

        @Override
        public int indexOf(Object o) {
            return 0;
        }

        @Override
        public int lastIndexOf(Object o) {
            return 0;
        }

        @Override
        public ListIterator<VuforiaTrackable> listIterator() {
            return null;
        }

        @Override
        public ListIterator<VuforiaTrackable> listIterator(int index) {
            return null;
        }

        @Override
        public List<VuforiaTrackable> subList(int fromIndex, int toIndex) {
            return null;
        }
    };



    //
    //GYRO
    //
    public String xyz = "z";
    public BNO055IMU imu;
    public Orientation angles = new Orientation();
    public Acceleration gravity;
    BNO055IMU.Parameters p = new BNO055IMU.Parameters();

    //GYRO CONSTANTS
    //public
    public double turnSpeed = .4;

    //private
    private double gyroTurnThreshold = .5;
    private double gyroTurnModLeft = .025;
    private double gyroTurnModRight = -.015;

    @Override
    public void runOpMode()
    {

    }

    //
    //INIT FUNCTIONS
    //
    public void initVuforia()
    {
        //TODO: FIX
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


        telemetry.addData("Test", "1");
        sleep(500);


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



        telemetry.addData("Test", "2");
        sleep(500);


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

        telemetry.addData("Test", "3");
        sleep(500);


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

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */


        telemetry.addData("Test", "4");
        sleep(500);


        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


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

    //
    //VUFORIA
    //
    public void vuforia(List<VuforiaTrackable> allTrackables, VuforiaTrackables targetsUltimateGoal, double timeoutS)
    {
        //TODO: FIX
        runtime.reset();
        targetsUltimateGoal.activate();
        while (runtime.seconds() < timeoutS)
        {

            // check all the trackable targets to see which one (if any) is visible.
            telemetry.addData("Test", "5");
            sleep(100);
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


        telemetry.addData("Test", "6");
        sleep(500);


        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    //
    //GYRO
    //
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

    public double readAngle(String xyz)
    {
        Orientation angles;
        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

    //
    //DRIVING
    //
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

    //
    //TURNING
    //
    public void turnToPosition (double pos, String xyz, double topPower, double timeoutS) {
        //COUNTER CLOCKWISE IS POSITIVE; CLOCKWISE IS NEGATIVE
        //TODO: FIX RIGHT TURN

        stopAndReset();
        double originalAngle = readAngle(xyz);
        double target = pos;

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        double degreesTurned;
        do {
            //salient values
            angle = readAngle(xyz);
            error = angle - target;
            degreesTurned = angle - originalAngle;
            powerScaled = topPower * Math.abs(error/90) * pidMultiplierTurning(error);

            //prevents extreme slowing towards end of turn
            if(-6 < error && error < 0)
            {
                powerScaled += gyroTurnModLeft;
            }
            else if (0 < error && error < 6)
            {
                powerScaled += gyroTurnModRight;
            }

            //telementry
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.addData("degrees turned", degreesTurned);
            telemetry.addData("target", target);
            telemetry.update();

            //direction handling
            if (error > 0)
            {
                normalDrive(powerScaled, -powerScaled);
            }
            else if (error < 0)
            {

                normalDrive(-powerScaled, powerScaled);
            }

            updateAngles();
        }
        while (opModeIsActive() && (Math.abs(error) > gyroTurnThreshold) && (runtime.seconds() < timeoutS));

        //stop turning and reset for next action
        normalDrive(0, 0);
        stopAndReset();
        updateAngles();

    }

    public void turnDegrees(double degrees, String xyz, double topPower, double timeoutS)
    {
        //COUNTER CLOCKWISE IS POSITIVE; CLOCKWISE IS NEGATIVE
        //TODO: TEST

        double originalAngle = readAngle(xyz);
        double angle = originalAngle + degrees;

        turnToPosition(angle, xyz, topPower, timeoutS);
    }

    //
    //MISC
    //
    public double pidMultiplierTurning(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        double C = .001;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }

    public void pathComplete(int millisec)
    {
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(millisec);
    }

}