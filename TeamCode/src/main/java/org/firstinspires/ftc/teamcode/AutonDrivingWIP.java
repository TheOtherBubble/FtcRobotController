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

public class AutonDrivingWIP extends LinearOpMode {

    //
    //OPMODE MEMBERS
    public Hardware robot = new Hardware();
    public ElapsedTime runtime = new ElapsedTime();



    //
    //MEASURING CONSTANTS
    //
    static final double     COUNTS_PER_MOTOR_REV = 383.6;    // Currently: Andymark Neverest 20
    static final double     DRIVE_GEAR_REDUCTION = .66666;    // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable


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
    public double smallTurnSpeed = .8;
    public double NORTH = 0;
    public double SOUTH = 180;
    public double EAST = 90;
    public double WEST = -90;

    //private
    private double gyroTurnThreshold = 1.4375;
    private double gyroTurnModLeft = .025;
    private double gyroTurnModRight = -.015;

    //
    //DRIVE
    //
    //public
    public double gyroDriveSpeed = .4;
    public double gyroDriveSpeedStrafe = .5;
    private double gyroStrafeAdj = 2;

    //private
    private double gyroDriveThreshold = .7;

    //
    //STRAFE
    //
    public boolean right = true;
    public boolean left = false;

    //balance reduction: add to this if the back of the robot is faster than the front, subtract for opposite
    //more balance: add to this if the robot is driving slightly backwards, subtract for opposite
    private double rightBalRed = .08;
    private double rightMoreBal = .4;

    private double leftBalRed = .05;
    private double leftMoreBal = .4;

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

        // Next, translate the came a lens to where it is on the robot.
        // In this example, it is c ntered (left to right), but forward of the middle of the robot, and above ground level.
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
        robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe (int iterations, double speed, boolean isRight, double balanceReduction, double milliseconds, double moreBalance, double Angle)
    {
        //LEGACY STRAFE

        //balance reduction: add to this if the back of the robot is faster than the front, subtract for opposite
        //more balance: add to this if the robot is driving slightly backwards, subtract for opposite
        stopAndReset();
        runtime.reset();
        //int     newRightTarget;
        double fLSpeed = 0;
        double fRSpeed = 0;
        double bLSpeed = 0;
        double bRSpeed = 0;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            for(int i = 0; i < iterations; i++) {
                ElapsedTime clock = new ElapsedTime();
//                moveCounts = (int) (distance * COUNTS_PER_INCH)/10;
//                fLTarget = (robot.fLMotor.getCurrentPosition() + moveCounts);
//                fRTarget = (robot.fRMotor.getCurrentPosition() + moveCounts);
//                bLTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);
//                bRTarget = (robot.bLMotor.getCurrentPosition() + moveCounts);
//
//                // Set Target and Turn On RUN_TO_POSITION
//                robot.fLMotor.setTargetPosition(fLTarget);
//                robot.fRMotor.setTargetPosition(-fRTarget);
//                robot.bLMotor.setTargetPosition(-bLTarget);
//                robot.bRMotor.setTargetPosition(bRTarget);


                robot.fLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                // start motion.
                //gyroDriveSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
                while(clock.milliseconds() < milliseconds)
                {
                    fLSpeed = speed;
                    fRSpeed = speed;
                    bLSpeed = speed;
                    bRSpeed = speed;

                    if(isRight)
                    {
                        bRSpeed -= balanceReduction;
                        fLSpeed += balanceReduction;
                        fRSpeed += balanceReduction - moreBalance;
                        bLSpeed -= balanceReduction - moreBalance;

                        bLSpeed *= -1;
                        fRSpeed *= -1;

                    }

                    else
                    {
                        bRSpeed -= balanceReduction - moreBalance;
                        fLSpeed += balanceReduction - moreBalance;
                        fRSpeed += balanceReduction;
                        bLSpeed -= balanceReduction;

                        fLSpeed *= -1;
                        bRSpeed *= -1;

                    }

                    double max = Math.max(Math.max(fLSpeed, fRSpeed), Math.max(bLSpeed, bRSpeed));
                    if(max > 1)
                    {
                        fLSpeed/=max;
                        fRSpeed/=max;
                        bLSpeed/=max;
                        bRSpeed/=max;
                    }

                    robot.fLMotor.setPower(fLSpeed);
                    robot.fRMotor.setPower(fRSpeed);
                    robot.bLMotor.setPower(bLSpeed);
                    robot.bRMotor.setPower(bRSpeed);

                    telemetry.addData("fL Speed", fLSpeed);
                    telemetry.addData("fR Speed", fRSpeed);
                    telemetry.addData("bL Speed", bLSpeed);
                    telemetry.addData("bR Speed", bRSpeed);
                    telemetry.addData("milliseconds", milliseconds);
                    telemetry.addData("clock", clock.milliseconds());
                    telemetry.addData("iteration", i);
                    telemetry.update();

                }

                //slow down bc jerk causes drift and turning
                double inc = .85;
                for(int j = 0; j < 15; j++)
                {
                    robot.fLMotor.setPower(robot.fLMotor.getPower()*inc);
                    robot.fRMotor.setPower(robot.fRMotor.getPower()*inc);
                    robot.bLMotor.setPower(robot.bLMotor.getPower()*inc);
                    robot.bRMotor.setPower(robot.bRMotor.getPower()*inc);
                }
                normalDrive(0, 0); // stops it after 1 second
                turnToPosition(Angle, "z", smallTurnSpeed, 500, true);
                //turnToPosition(-angle, "z", turnSpeed, 4); //corrects at the end of each motion set
                sleep(300);
                //telemetry.addData("Target", "%7d:%7d:%7d:%7d", fLTarget, fRTarget, bLTarget, bRTarget);
            }

            // Stop all motion;
            normalDrive(0, 0);

            //correct for drift during drive
            //turnToPosition(-angle, "z", turnSpeed, 3);

            // Turn off RUN_TO_POSITION
            stopAndReset();
        }
    }

    //
    //TURNING
    //
    public void turnToPosition (double pos, String xyz, double topPower, double timeoutS, boolean gyroDrive) {
        //COUNTER CLOCKWISE IS POSITIVE; CLOCKWISE IS NEGATIVE

        stopAndReset();
        double originalAngle = readAngle(xyz);
        double target = pos;

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        double degreesTurned;

        /*if(target < 0)
        {
            target += 3;
        }
        else if(target > 0)
        {
            target += .5;
        }*/
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
        while (opModeIsActive() && ((Math.abs(error) > gyroTurnThreshold) || (gyroDrive && ((Math.abs(error) >= 1.75) && Math.abs(error) <= 2.25))) && (runtime.seconds() < timeoutS));

        //stop turning and reset for next action
        normalDrive(0, 0);
        stopAndReset();
        updateAngles();

    }

    public void turnDegrees(double degrees, String xyz, double topPower, double timeoutS)
    {
        //COUNTER CLOCKWISE IS POSITIVE; CLOCKWISE IS NEGATIVE

        double originalAngle = readAngle(xyz);
        double angle = originalAngle + degrees;

        turnToPosition(angle, xyz, topPower, timeoutS, false);
    }

    public void gyroDrive (double inches, char direction, double angle, double speed, double timeoutS)
    {

        runtime.reset();
        stopAndReset();

        int fLTarget = 0;
        int fRTarget = 0;
        int bLTarget = 0;
        int bRTarget = 0;

        double fLSpeed = speed;
        double fRSpeed = speed;
        double bLSpeed = speed;
        double bRSpeed = speed;




        //int     newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        int decelerateThreshold = 170;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(inches * COUNTS_PER_INCH);
            if(direction == 'f')
            {
                fLTarget = robot.fLMotor.getCurrentPosition() + moveCounts;
                fRTarget = robot.fRMotor.getCurrentPosition() + moveCounts;
                bLTarget = robot.bLMotor.getCurrentPosition() + moveCounts;
                bRTarget = robot.bLMotor.getCurrentPosition() + moveCounts;
                robot.fLMotor.setTargetPosition(fLTarget);
                robot.fRMotor.setTargetPosition(fRTarget);
                robot.bLMotor.setTargetPosition(bLTarget);
                robot.bRMotor.setTargetPosition(bRTarget);
            }
            else if(direction == 'b')
            {
                fLTarget = robot.fLMotor.getCurrentPosition() - moveCounts;
                fRTarget = robot.fRMotor.getCurrentPosition() - moveCounts;
                bLTarget = robot.bLMotor.getCurrentPosition() - moveCounts;
                bRTarget = robot.bLMotor.getCurrentPosition() - moveCounts;
                robot.fLMotor.setTargetPosition(fLTarget);
                robot.fRMotor.setTargetPosition(fRTarget);
                robot.bLMotor.setTargetPosition(bLTarget);
                robot.bRMotor.setTargetPosition(bRTarget);
            }
            else if(direction == 'l')
            {
                double balanceReduction = leftBalRed;
                double moreBalance = leftMoreBal;

                moveCounts += gyroDriveSpeedStrafe; //janky

                bRTarget = robot.bRMotor.getCurrentPosition() - moveCounts;
                bLTarget = robot.bLMotor.getCurrentPosition() + moveCounts;
                fRTarget = robot.fRMotor.getCurrentPosition() + moveCounts;
                fLTarget = robot.fLMotor.getCurrentPosition() - moveCounts;

                bRSpeed -= balanceReduction - moreBalance;
                fLSpeed += balanceReduction - moreBalance;
                fRSpeed += balanceReduction;
                bLSpeed -= balanceReduction;

                bRSpeed *= -1;
                fLSpeed *= -1;

                robot.bRMotor.setTargetPosition(bRTarget);
                robot.bLMotor.setTargetPosition(bLTarget);
                robot.fRMotor.setTargetPosition(fRTarget);
                robot.fLMotor.setTargetPosition(fLTarget);

            }
            else if(direction == 'r')
            {
                double balanceReduction = rightBalRed;
                double moreBalance = rightMoreBal;

                moveCounts += gyroDriveSpeedStrafe; //janky
                fLTarget = robot.fLMotor.getCurrentPosition() + moveCounts;
                fRTarget = robot.fRMotor.getCurrentPosition() - moveCounts;
                bLTarget = robot.bLMotor.getCurrentPosition() - moveCounts;
                bRTarget = robot.bLMotor.getCurrentPosition() + moveCounts;

                bRSpeed -= balanceReduction;
                fLSpeed += balanceReduction;
                fRSpeed += balanceReduction - moreBalance;
                bLSpeed -= balanceReduction - moreBalance;

                bLSpeed *= -1;
                fRSpeed *= -1;

                robot.fLMotor.setTargetPosition(fLTarget);
                robot.fRMotor.setTargetPosition(fRTarget);
                robot.bLMotor.setTargetPosition(bLTarget);
                robot.bRMotor.setTargetPosition(bRTarget);

            }


            // Set Target and Turn On RUN_TO_POSITION
//            robot.fLMotor.setTargetPosition(fLTarget);
//            robot.fRMotor.setTargetPosition(fRTarget);
//            robot.bLMotor.setTargetPosition(bLTarget);
//            robot.bRMotor.setTargetPosition(bRTarget);


            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            //gyroDriveSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.fLMotor.setPower(fLSpeed);
            robot.fRMotor.setPower(fRSpeed);
            robot.bLMotor.setPower(bLSpeed);
            robot.bRMotor.setPower(bRSpeed);

            // keep looping while we are still active, and BOTH motors are running.
            //WILL NOT WORK WITH STRAFE
            //TODO: MAKE STRAFE COMPATIBLE (BOTH WHILE LOOP AND STEER FUNCTION)
            while (opModeIsActive() &&
                    (robot.fLMotor.isBusy() && robot.fRMotor.isBusy() && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()) && runtime.seconds() < timeoutS) {

                // adjust relative speed based on heading error.
                error = getError(angle);

                if(Math.abs(error) < gyroDriveThreshold)
                {
                    error = 0;
                }

                //changes powers in order to correct for turning while driving
                steer = -getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (inches < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                //makes first 100 counts faster bc drive takes a lot of time to accelerate
                /*if(initBoost) {
                    if (Math.abs(robot.fLMotor.getCurrentPosition() - fLInit) < 100 && Math.abs(robot.bLMotor.getCurrentPosition() - bLInit) < 100)
                    {
                        leftSpeed += gyroDriveInitBoost;
                    }
                    if (Math.abs(robot.fRMotor.getCurrentPosition() - fRInit) < 100 && Math.abs(robot.bRMotor.getCurrentPosition() - bRInit) < 100)
                    {
                        rightSpeed += gyroDriveInitBoost;
                    }
                }*/
                if(Math.abs(fLTarget - robot.fLMotor.getCurrentPosition()) < decelerateThreshold && Math.abs(fRTarget - robot.fRMotor.getCurrentPosition()) < decelerateThreshold && Math.abs(bLTarget - robot.bLMotor.getCurrentPosition()) < decelerateThreshold && Math.abs(bRTarget - robot.bRMotor.getCurrentPosition()) < decelerateThreshold)
                {
                    leftSpeed /= 3;
                    rightSpeed /= 3;
                }
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.fLMotor.setPower(leftSpeed);
                robot.fRMotor.setPower(rightSpeed);
                robot.bLMotor.setPower(leftSpeed);
                robot.bRMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      fLTarget,  fRTarget, bLTarget, bRTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.fLMotor.getCurrentPosition(),
                        robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("Angle", angle);
                telemetry.update();
            }

            // Stop all motion;
            normalDrive(0, 0);

            //attempted remedy for slow/unnecessary turn to position(goal was to to allow it to reset from stopping before it read angle
            //sleep(333);

            //correct for drift in angle (Adjusted because it was consistently attempting to turn with an error of close to 2 when no turn was necessary
            if(getError(0) <= 2.25 || getError(0) >= 1.75)
            {
                turnToPosition(angle, xyz, smallTurnSpeed, 1000, true);
            }

            // Turn off RUN_TO_POSITION
            stopAndReset();
        }
    }

    //
    //MISC
    //
    public double getError(double targetAngle)
    {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - readAngle("z");
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double pidMultiplierTurning(double error)
    {
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