package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * The Main Iterative TeleOp for Thunderbots Too 2025
 * More documentation needs to be added in the future
 * @see HardwareManagerNew
 * @version 2026.01.07.0
 * @author Max & Devon
 */
@TeleOp(name = "Main Iterative TeleOp New", group = "iterative")
public class MainTeleOpNew extends OpMode {

    private static final String VERSION = "2025.12.13.0";

    //The HardwareManager instance for physical Robot interactions
    private HardwareManagerNew hardwareManager = null;


    //Stores the time running for further reference, eg telemetry
    private ElapsedTime runtime = null;

    //Stores robot states
    private boolean flyWheelOn = false;
    private boolean close = false;
    private boolean drive = false;


    private ElapsedTime shootTime = null;


    private int ballsToShoot = 0;

    @Override
    public void init() {
        this.telemetry.addData("Status", "initializing");
        this.telemetry.update();

        hardwareManager = new HardwareManagerNew(this);

        hardwareManager.init();
        this.flyWheelOn = false;
        this.ballsToShoot = 0;
    }

    @Override
    public void init_loop() {
        this.telemetry.addData("Status", "initialized");
        this.telemetry.addData("Info", "Main Iterative TeleOp V"+ VERSION +"; H"+HardwareManagerNew.VERSION);
        this.telemetry.update();
    }

    @Override
    public void start() {
        this.telemetry.addData("Status", "starting");
        this.telemetry.update();

        this.runtime = new ElapsedTime();
    }

    @Override
    public void loop() {
        hardwareManager.getAlliance();
        //Emergency break
        if (gamepad1.back) this.terminateOpModeNow();


        this.telemetry.addData("Status", "running for " + this.runtime.seconds() + " seconds");
        this.telemetry.addData("Info", "Main Iterative TeleOp V" + VERSION + "; H" + HardwareManagerNew.VERSION);

        if (gamepad1.optionsWasPressed()){
            drive = !drive;
        }

        this.telemetry.addData("Drive Mode", drive ? "Standard" : "Field Relative");
        if (drive){
            this.hardwareManager.axialLateralYaw(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } if (!drive){
            this.hardwareManager.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.rightBumperWasPressed()) this.flyWheelOn = !this.flyWheelOn;

        if (gamepad1.dpadDownWasPressed()) this.close = !close;

        this.telemetry.addData("Position", close ? "Close" : "Far");

        boolean outake = gamepad1.left_bumper;
        boolean intake = gamepad1.left_trigger > 0.1;

        hardwareManager.manualShoot(gamepad1.right_trigger > 0.2);

        if (gamepad1.xWasPressed()){
            this.hardwareManager.odometryInitRed();
        }
        if (gamepad1.yWasPressed()){
            this.hardwareManager.odometryInitBlue();
        }

        if (gamepad1.dpadRightWasPressed()){
            this.hardwareManager.resetOdoRed();
        }

        boolean active = gamepad1.dpadUpWasPressed();

        this.hardwareManager.odometry.update();

        this.hardwareManager.setRobotPosition();

        this.telemetry.addData("Alliance", hardwareManager.getAlliance() ? "red" : "blue");

        this.hardwareManager.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        this.hardwareManager.axialLateralYaw(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        this.hardwareManager.fly(this.flyWheelOn, close);
        this.hardwareManager.getPosition(close);
        if (intake || outake) hardwareManager.intake(true, intake);
        else hardwareManager.intake(false);
        this.hardwareManager.cameraStream(active);

        this.hardwareManager.getBlockPos();
        this.hardwareManager.odometryTelemetry();
        telemetry.addData("Robot Angle", this.hardwareManager.getIMU(AngleUnit.DEGREES));
        this.hardwareManager.driveTelemetry();
        this.hardwareManager.telemetryAprilTag();

    }

    @Override
    public void stop() {
        this.telemetry.addData("Status", "stopping after " + ((this.runtime != null) ? this.runtime.seconds() : 0) + " seconds");
    }
}
