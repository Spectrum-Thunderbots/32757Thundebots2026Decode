package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

/**
 * The HardwareManager abstracts interaction with the hardware into another
 * class and simplifies interactions.
 * More documentation needs to be added in the future
 * WARNING Many features temporarily deactivated in the version!!
 * @version 2025.12.07.0
 * @author Max & Devon (Yay dream-team)
 */
public class HardwareManagerNew {

    public static final String VERSION = "2026.01.07.0";

    private enum Mode {
        ALL
    }


    public enum IntakeMode { //NEW
        INTAKE,
        OUTTAKE,
        PAUSE
    }

    //At the moment these values are simplified - subject to change

    public boolean redAlliance = true;

    public final static double SHOOT_TIME = 750; //The time it usually takes to shoot a single artefact
    public final static double TIME_TO_REACTIVATE = 100; //The time it takes for the fly wheels to be able to shoot another bll after shooting
    public final static double TIME_TO_ACTIVATE = 1000; //The time it takes for the fly wheels to reach optimal speed

    private final static double OPEN_BLOCKER_POS = .6;
    private final static double CLOSED_BLOCKER_POS = .4;

    private final static double INTAKE_POWER = 1.0; //NEW
    private final static double OUTTAKE_POWER = 1.0; //Higher value = faster movement, direction is being handled automatically //NEW

    private final static double FAR_FLY_POWER = 0.9;
    private final static double CLOSE_FLY_POWER = 0.6;

    private final static double FLY_VELOCITY = 2250;
    private final static double CLOSE_FLY_VELOCITY = 1700;
    private final static double CLOSE_FLY_AUTO_VELOCITY = 1650;

    public final static int MAX_BALL_CAPACITY = 3;

    private final static double DRIVE_SPEED = 0.6;
    private final static double TURN_SPEED = 0.5;

    private static final double COUNTS_PER_MOTOR_REV = 415.2;
    private static final double WHEEL_DIAMETER_INCHES = 4.094488;
    public static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * 3.1416);

    private Mode initMode = null;
    private boolean encoder = false;
    private ElapsedTime runtime = null;
    private boolean fly = false;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor intakeMotor = null;
    private DcMotorEx flyMotor = null;

    public GoBildaPinpointDriver odometry = null;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
    private LinearOpMode linearOpMode = null;
    private final Gamepad gamepad;

    private IMU imu = null;

    private Servo blockShootServo = null;


    //private RevBlinkinLedDriver blinker = null;

    private void updateTelemetry(String caption, String info){
        if (this.telemetry == null) return;
        this.telemetry.addData(caption, info);
    }

    private void updateTelemetry(String info){
        if (this.telemetry == null) return;
        this.telemetry.addData(">", info);
    }

    private void blink(RevBlinkinLedDriver.BlinkinPattern pattern){
        this.allInitCheck();
        if (pattern == null) throw new IllegalArgumentException("Pattern must be specified!");
        //this.blinker.setPattern(pattern);
    }

    //TODO: Make sure all methods use this
    private void allInitCheck() {
        if (this.initMode != Mode.ALL) throw new IllegalStateException("HardwareManager needs to be initialized before usage. Use HardwareManager.init");
    }

    private double rangeCheck(double value, double min, double max){
        if (value < min){
            value = min;
        } if (value > max){
            value = max;
        }
        return value;
    }

    public double rangeCheckAngle(double value, double min, double max){
        while (value <= min){
            value += 360;
        } while (value > max){
            value -= 360;
        }
        return value;
    }

    public HardwareManagerNew(OpMode opMode){
        if (opMode == null) throw new IllegalArgumentException("opMode cannot be null");
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad = opMode.gamepad1;
        if (opMode instanceof LinearOpMode) this.linearOpMode = (LinearOpMode) opMode;
    }

    public void init(){
        this.init(false, false);
    }

    public void init(boolean driveEncoder) {
        this.init(driveEncoder, false);
    }

    public void init(boolean driveEncoder, boolean advancedSecurity) {
        this.encoder = driveEncoder;

        this.frontLeftDrive = this.hardwareMap.get(DcMotor.class, "frontLeftDrive");
        this.frontRightDrive = this.hardwareMap.get(DcMotor.class, "frontRightDrive");
        this.backLeftDrive = this.hardwareMap.get(DcMotor.class, "backLeftDrive");
        this.backRightDrive = this.hardwareMap.get(DcMotor.class, "backRightDrive");

        this.intakeMotor = this.hardwareMap.get(DcMotor.class, "intakeMotor");
        this.flyMotor = this.hardwareMap.get(DcMotorEx.class, "flyMotor");

        this.blockShootServo = this.hardwareMap.get(Servo.class, "blockShootServo");

        this.imu = this.hardwareMap.get(IMU.class, "imu");

        this.odometry = this.hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );

        imu.initialize(new IMU.Parameters(RevOrientation));


        this.frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        this.frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        this.backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        this.backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        this.intakeMotor.setDirection(DcMotor.Direction.FORWARD); //INTAKE
        this.flyMotor.setDirection(DcMotorEx.Direction.FORWARD);

        this.blockShootServo.setDirection(Servo.Direction.REVERSE);
        if (driveEncoder){
            this.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            this.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.flyMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.flyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.flyMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.blockPos(false);

        this.initMode = Mode.ALL;

        initAprilTag();
    }

    public void odometryInitRed(){
        odometry.setOffsets(116.75, 96.75, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.recalibrateIMU();
        odometry.setPosY(9, DistanceUnit.INCH);
        odometry.setPosX(9, DistanceUnit.INCH);
        imu.resetYaw();
    }

    public void odometryInitBlue(){
        odometry.setOffsets(116.75, 96.75, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.recalibrateIMU();
        odometry.setPosY(9, DistanceUnit.INCH);
        odometry.setPosX(135, DistanceUnit.INCH);
        imu.resetYaw();
    }

    public void resetOdoRed(){
        odometry.recalibrateIMU();
        odometry.setPosY(9, DistanceUnit.INCH);
        odometry.setPosX(9, DistanceUnit.INCH);
        imu.resetYaw();
    }

    public void resetOdoBlue(){
        odometry.recalibrateIMU();
        odometry.setPosY(9, DistanceUnit.INCH);
        odometry.setPosX(135, DistanceUnit.INCH);
        imu.resetYaw();
    }


    public void odometryTelemetry(){
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", odometry.getPosX(DistanceUnit.MM), odometry.getPosY(DistanceUnit.MM), odometry.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odometry.getVelX(DistanceUnit.MM), odometry.getVelY(DistanceUnit.MM), odometry.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
    }

    private void setDriveMode(DcMotor.RunMode mode){
        this.frontLeftDrive.setMode(mode);
        this.frontRightDrive.setMode(mode);
        this.backLeftDrive.setMode(mode);
        this.backRightDrive.setMode(mode);
    }

    private void setSafePower(DcMotor motor, double targetPower){
        final double SLEW_RATE = 0.2;
        double currentPower = motor.getPower();

        double desiredChange = targetPower - currentPower;
        double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));

        motor.setPower(currentPower + limitedChange);
    }

    public void individualPower(double frontLeft, double frontRight, double backLeft, double backRight){
        this.allInitCheck();

        for (double value : new double[] {frontLeft, frontRight, backLeft, backRight}){
            this.rangeCheck(value, -1, 1);
        }


        this.setSafePower(this.frontLeftDrive, frontLeft);
        this.setSafePower(this.frontRightDrive, frontRight);
        this.setSafePower(this.backLeftDrive, backLeft);
        this.setSafePower(this.backRightDrive, backRight);
    }

    public void axialLateralYaw(double axial, double lateral, double yaw){

        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        this.individualPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void driveFieldRelative(double axial, double lateral, double yaw) {
        double theta = Math.atan2(axial, lateral);
        double r = Math.hypot(lateral, axial);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        axialLateralYaw(newForward, newRight, yaw);
    }

    public void imuReset(){
        imu.resetYaw();
    }

    public void driveFieldRelativeBlue(double axial, double lateral, double yaw) {
        double theta = Math.atan2(axial, lateral);
        double r = Math.hypot(lateral, axial);

        theta = AngleUnit.normalizeRadians(theta -
                odometry.getHeading(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        axialLateralYaw(newForward, newRight, yaw);
    }

    public void driveTelemetry(){
        String drive = String.format(Locale.US, "{FL: %.3f, FR: %.3f, BL: %.3f, BR: %.3f}", frontLeftDrive.getPower(), frontRightDrive.getPower(), backLeftDrive.getPower(), backLeftDrive.getPower());
        updateTelemetry("Drive:", drive);
    }

    private void encoderDrive(double speed, double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches, double timeout, boolean fly, boolean close){
        this.allInitCheck();
        if (!this.encoder) throw new IllegalStateException("Encoder are required for encoderDrive!");
        if (this.linearOpMode == null) throw new IllegalStateException("encoderDrive only works with linearOpMode at the moment");
        this.rangeCheck(speed, 0, 1);

        int frontLeftDriveTarget;
        int frontRightDriveTarget;
        int backLeftDriveTarget;
        int backRightDriveTarget;

        if (this.linearOpMode.opModeIsActive()){
            frontLeftDriveTarget = this.frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            frontRightDriveTarget = this.frontRightDrive.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            backLeftDriveTarget = this.backLeftDrive.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            backRightDriveTarget = this.backRightDrive.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            this.frontLeftDrive.setTargetPosition(frontLeftDriveTarget);
            this.frontRightDrive.setTargetPosition(frontRightDriveTarget);
            this.backLeftDrive.setTargetPosition(backLeftDriveTarget);
            this.backRightDrive.setTargetPosition(backRightDriveTarget);

            this.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (this.runtime == null) this.runtime = new ElapsedTime();
            this.runtime.reset();


            this.frontLeftDrive.setPower(speed);
            this.frontRightDrive.setPower(speed);
            this.backLeftDrive.setPower(speed);
            this.backRightDrive.setPower(speed);

            while (this.linearOpMode.opModeIsActive() && (this.runtime.seconds() < timeout)
                    && (this.frontLeftDrive.isBusy() && this.frontRightDrive.isBusy()
                    && this.backLeftDrive.isBusy() && this.backRightDrive.isBusy())) {
                if (fly){
                    this.fly(true, close);
                }
            }

            this.individualPower(0, 0, 0, 0);
            this.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void encoderForwardDrive(double inches, double timeout, boolean fly, boolean close){
        this.encoderDrive(DRIVE_SPEED, inches, inches, inches, inches, timeout, fly, close);
    }

    public void encoderForwardDrive(double inches){
        this.encoderForwardDrive(inches, 100, false, false);
    }

    public void encoderForwardDrive(double inches, boolean close){
        this.encoderForwardDrive(inches, 100, true, close);
    }

    public void encoderTurnDrive(double inches, double timeout, boolean fly, boolean close){
        this.encoderDrive(TURN_SPEED, inches, -inches, inches, -inches, timeout, fly, close);
    }

    public void encoderTurnDrive(double inches){
        this.encoderTurnDrive(inches, 100, false, false);
    }

    public void encoderTurnDrive(double inches, boolean close){
        this.encoderTurnDrive(inches, 100, true, close);
    }

    private void setIntakePower(double power){
        this.allInitCheck();
        this.rangeCheck(power, -1, 1);
        this.intakeMotor.setPower(power);
    }

    public void setIntakeMode(IntakeMode intakeMode){ //NEW
        double power = 0;
        switch (intakeMode){
            case INTAKE:
                power = INTAKE_POWER;
                break;
            case OUTTAKE:
                power = -OUTTAKE_POWER;
                break;
            case PAUSE:
                power = 0;
                break;
        }

        this.setIntakePower(power);
    }

    public void intake(boolean active){
        this.intake(active, true);
    }

    public void intake(boolean active, boolean forward){ //NEW
        if (active) {
            if (forward) {
                this.setIntakeMode(IntakeMode.INTAKE);
            } else {
                this.setIntakeMode(IntakeMode.OUTTAKE);
            }
        } else {
            this.setIntakeMode(IntakeMode.PAUSE);
        }
    }

    public boolean isIntake(){
        this.allInitCheck();
        return this.intakeMotor.getPower() > 0;
    }

    public void fly(boolean active) {this.fly(active, false);}

    public void fly(boolean active, boolean close){
        this.allInitCheck();

        this.fly = active;

        double actualVelocity = this.flyMotor.getVelocity();
        this.updateTelemetry("Fly Wheel Velocity", Double.toString(actualVelocity));

        double positionVelocity = close ? (linearOpMode == null ? CLOSE_FLY_VELOCITY : CLOSE_FLY_AUTO_VELOCITY) : FLY_VELOCITY;

        double targetPower = active ? computeFlyTargetPower(positionVelocity, actualVelocity) : 0;

        this.flyMotor.setPower(targetPower);

        this.updateTelemetry("Fly Power", Double.toString(this.flyMotor.getPower()));
        this.blink(active ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }


    public void positionFly(boolean active){
        this.allInitCheck();

        this.fly = active;

        double actualVelocity = this.flyMotor.getVelocity();
        this.updateTelemetry("Fly Wheel Velocity", Double.toString(actualVelocity));



        double red = Math.hypot(odometry.getPosX(DistanceUnit.INCH) - 128, odometry.getPosY(DistanceUnit.INCH) - 129);
        double blue = Math.hypot(odometry.getPosX(DistanceUnit.INCH) - 128, odometry.getPosY(DistanceUnit.INCH) - 14);

        double distance = redAlliance ? red : blue;
        double positionVelocity = (-0.0153 * Math.pow(distance, 2)) + (11.9 * distance) + 974.8;

        double targetPower = active ? computeFlyTargetPower(positionVelocity, actualVelocity) : 0;

        this.flyMotor.setPower(targetPower);

        this.updateTelemetry("Fly Power", Double.toString(this.flyMotor.getPower()));
    }

    private static double computeFlyTargetPower(double targetVelocity, double actualVelocity) {
        double f = 0.000424;
        double p = 0.000365;

        double targetPower = (f * targetVelocity) + (p * (targetVelocity - actualVelocity));

        return targetPower;
    }



    public boolean isFly(){
        return this.fly;
    }

    public void stopDrive() {
        if (this.frontLeftDrive != null) this.frontLeftDrive.setPower(0);
        if (this.frontRightDrive != null) this.frontRightDrive.setPower(0);
        if (this.backLeftDrive != null) this.backLeftDrive.setPower(0);
        if (this.backRightDrive != null) this.backRightDrive.setPower(0);
    }

    public void blockPos(boolean active){
        this.blockShootServo.setPosition(!active ? OPEN_BLOCKER_POS : CLOSED_BLOCKER_POS);
    }

    public void manualShoot(boolean active){
        this.blockPos(active);
        this.intake(active,active);
    }

    public void getBlockPos(){
        telemetry.addData("ServoPos", this.blockShootServo.getPosition());
    }

    public void stop(){
        this.stopDrive();
        this.fly(false);
        this.intake(false);
        this.manualShoot(false);
    }

    public double getIMU(AngleUnit angle) {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        return Yaw;
    }

    public boolean getPosition(boolean close){
        return close;
    }
    public boolean getAlliance(){
        return this.redAlliance;
    }

    public void IMUTELEMETRY(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
    }

    private final Position cameraPosition = new Position(DistanceUnit.MM,
            0, 139.84486, 156.630, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 60, 0, 0);

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    boolean USE_WEBCAM = true;

    public void cameraStream(boolean active) {
        if (active) visionPortal.resumeStreaming();
        else visionPortal.stopStreaming();

        telemetry.addData("Camera", active ? "On" : "Off" );
    }

    public void initAprilTag() {


        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }
    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        updateTelemetry("# AprilTags Detected", String.valueOf(currentDetections.size()));

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                updateTelemetry(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                if (!detection.metadata.name.contains("Obelisk")) {
                    updateTelemetry(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    updateTelemetry(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                updateTelemetry(String.format("\n==== (ID %d) Unknown", detection.id));
                updateTelemetry(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        updateTelemetry("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        updateTelemetry("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }

    public void setRobotPosition(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                odometry.setPosX(detection.robotPose.getPosition().x, DistanceUnit.INCH);
                odometry.setPosY(detection.robotPose.getPosition().y, DistanceUnit.INCH);
                odometry.setHeading(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES), AngleUnit.DEGREES);
            }
        }
    }


}
