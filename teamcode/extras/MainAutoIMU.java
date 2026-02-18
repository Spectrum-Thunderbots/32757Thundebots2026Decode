package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.main.HardwareManagerNew;

@Autonomous(name="IMU Main Linear Autonomous IMU")
public class MainAutoIMU extends LinearOpMode {

    private static final String VERSION = "2025.12.13.3";
    private static final String INFO = "Main Auto V"+VERSION;
    private boolean redAlliance;
    private boolean closePosition;
    private int balls;

    private boolean fly;

    private HardwareManagerNew hardwareManager = null;

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry.addData("Status", "Initializing");
        this.telemetry.addData("Info", INFO);
        this.telemetry.update();

        this.hardwareManager = new HardwareManagerNew(this);
        this.hardwareManager.init(true);

        this.telemetry.addData("Status", "Initializing - Action required");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Important", "Please select alliance");
        this.telemetry.addData("Y", "Red");
        this.telemetry.addData("A", "Blue");
        this.telemetry.update();

        /*while (this.gamepad1.y || this.gamepad1.a) {} //Busy waiting :-(
        while (!(this.gamepad1.y || this.gamepad1.a) ) {}
        this.redAlliance = this.gamepad1.y;
        while (this.gamepad1.y || this.gamepad1.a) {}*/

        while (!(this.gamepad1.yWasPressed() || this.gamepad1.aWasPressed() || this.isStopRequested())) {}
        if (this.isStopRequested()) return;
        this.redAlliance = this.gamepad1.y;

        this.telemetry.addData("Status", "Initializing - Action required");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Important", "Please select position");
        this.telemetry.addData("Y", "Close position");
        this.telemetry.addData("A", "Far Position");
        this.telemetry.update();

        /*while (this.gamepad1.y || this.gamepad1.a) {}
        while (!(this.gamepad1.y || this.gamepad1.a)) {}
        this.closePosition = this.gamepad1.y;
        while (this.gamepad1.y || this.gamepad1.a) {}*/

        while (!(this.gamepad1.yWasPressed() || this.gamepad1.aWasPressed() || this.isStopRequested())) {}
        if (this.isStopRequested()) return;
        this.closePosition = this.gamepad1.y;

        this.telemetry.addData("Status", "Initializing - Action required");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Important", "Please select a goal");
        this.telemetry.addData("Y", "9 ball");
        this.telemetry.addData("X", "6 balls");
        this.telemetry.addData("A", "3 balls");
        this.telemetry.update();

        /*while (!(this.gamepad1.y || this.gamepad1.x || this.gamepad1.a)) {}
        this.balls = this.gamepad1.y ? 9 : (this.gamepad1.x ? 6 : 3);*/

        while (!(this.gamepad1.yWasPressed() || this.gamepad1.xWasPressed() || this.gamepad1.aWasPressed() || this.isStopRequested())) {}
        if (this.isStopRequested()) return;
        this.balls = this.gamepad1.y ? 9 : (this.gamepad1.x ? 6 : 3);


        this.telemetry.addData("Status", "Initialized");
        this.telemetry.addData("Info", INFO);
        this.telemetry.addData("Mode", (this.redAlliance ? "Red" : "Blue") + "Alliance;" + (this.closePosition ? "Close" : "Far") + "Position;" + this.balls + " balls");
        //telemetry.addData("Robot Angle", this.hardwareManager.getIMU(AngleUnit.DEGREES));
        this.telemetry.update();


        this.hardwareManager.resetOdoRed();
        this.hardwareManager.getAlliance();
        this.hardwareManager.getPosition(closePosition);

        // this.hardwareManager.cameraStream(true);
        this.hardwareManager.blockPos(false);

        waitForStart(); }}


      /*  if (!this.redAlliance && !this.closePosition){
            prepShootFar();
            turnToAngle(-21);
            autoAllShoot();
            turnToAngle(0);
            if (this.balls > 3) {
                drive(30);
                turnToAngle(90);
                this.hardwareManager.intake(true);
                drive(-64);
                this.hardwareManager.intake(false);
                drive(64);
                turnToAngle(0);
                prepShootFar();
                drive(-30);
                turnToAngle(-21);
                autoAllShoot();
                turnToAngle(0);
                if (this.balls > 6){
                    drive(54);
                    turnToAngle(90);
                    this.hardwareManager.intake(true);
                    drive(-64);
                    this.hardwareManager.intake(false);
                    drive(64);
                    turnToAngle(0);
                    prepShootFar();
                    drive(-54);
                    turnToAngle(-21);
                    autoAllShoot();
                    turnToAngle(0);
                }
            }
            drive(45);
            turnToAngle(-90);
        }
        if (this.redAlliance && !this.closePosition){
            prepShootFar();
            turnToAngle(21);
            autoAllShoot();
            turnToAngle(0);
            if (this.balls > 3) {
                drive(30);
                turnToAngle(-90);
                this.hardwareManager.intake(true);
                drive(-64);
                this.hardwareManager.intake(false);
                drive(64);
                turnToAngle(0);
                prepShootFar();
                drive(-30);
                turnToAngle(21);
                autoAllShoot();
                turnToAngle(0);
                if (this.balls > 6){
                    drive(54);
                    turnToAngle(-90);
                    this.hardwareManager.intake(true);
                    drive(-64);
                    this.hardwareManager.intake(false);
                    drive(64);
                    turnToAngle(0);
                    prepShootFar();
                    drive(-54);
                    turnToAngle(21);
                    autoAllShoot();
                    turnToAngle(0);
                }
            }
            drive(45);
            turnToAngle(90);
        }
        if (!this.redAlliance && this.closePosition) {
            prepShootClose();
            drive(-66);
            turnToAngle(-45);
            autoAllShoot();
            if (balls > 3){
                turnToAngle(90);
                this.hardwareManager.intake(true);
                drive(-52);
                this.hardwareManager.intake(false);
                prepShootClose();
                drive(52);
                turnToAngle(-45);
                autoAllShoot();
                if (balls > 6){
                    turnToAngle(0);
                    drive(-36);
                    turnToAngle(90);
                    this.hardwareManager.intake(true);
                    drive(-64);
                    this.hardwareManager.intake(false);
                    prepShootClose();
                    drive(64);
                    turnToAngle(0);
                    drive(36);
                    turnToAngle(-45);
                    autoAllShoot();
                }
            }
            turnToAngle(0);
            drive(-36);
        }
        if (this.redAlliance && this.closePosition) {
            prepShootClose();
            drive(-66);
            turnToAngle(45);
            autoAllShoot();
            if (balls > 3){
                turnToAngle(-90);
                this.hardwareManager.intake(true);
                drive(-52);
                this.hardwareManager.intake(false);
                prepShootClose();
                drive(52);
                turnToAngle(45);
                autoAllShoot();
                if (balls > 6){
                    turnToAngle(0);
                    drive(-36);
                    turnToAngle(-90);
                    this.hardwareManager.intake(true);
                    drive(-64);
                    this.hardwareManager.intake(false);
                    prepShootClose();
                    drive(64);
                    turnToAngle(0);
                    drive(36);
                    turnToAngle(45);
                    autoAllShoot();
                }
            }
            turnToAngle(0);
            drive(-36);
        }
    }


    @Deprecated private void prepShootClose(){
        prepShoot();
    }

    @Deprecated private void prepShootFar(){
        prepShoot();
    }

    private void prepShoot(){
        this.fly = true;
    }

    private void autoShootImpl(){
        this.hardwareManager.stopDrive();
        this.hardwareManager.blockPos(true);
        this.hardwareManager.intake(true);
        sleep((long) HardwareManagerNew.SHOOT_TIME);
        this.hardwareManager.intake(false);
        this.hardwareManager.blockPos(false);
        this.telemetry.update();
    }

    private void autoAllShoot(){
        for (int i = 0; i < HardwareManagerNew.MAX_BALL_CAPACITY; i++){
            this.autoShootImpl();
        }

        this.fly = false;
        this.hardwareManager.fly(false);
    }

    private void turn(double inches){
        if (this.fly) this.hardwareManager.encoderTurnDrive(inches, this.closePosition);
        else this.hardwareManager.encoderTurnDrive(inches);
    }

    private void drive(double inches){
        if (this.fly) this.hardwareManager.encoderForwardDrive(inches, this.closePosition);
        else this.hardwareManager.encoderForwardDrive(inches);
    }

    private void intake(boolean active){
        if (!active) intakeWait();
        this.hardwareManager.intake(active);
    }

    private void intakeWait(){
        this.hardwareManager.stopDrive();
        this.sleep(350);
    }


    public void turnToAngle(double angle) {
        double headingError = getSteeringCorrection(angle);

        while (opModeIsActive() && (Math.abs(headingError) > 1.0)) {

            double turnSpeed = getSteeringCorrection(angle);

        // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -0.5, 0.5);

        // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
        }
        moveRobot(0, 0);
    }
    public double getSteeringCorrection(double desiredHeading){
        double headingError = desiredHeading - this.hardwareManager.getIMU(AngleUnit.DEGREES);
        this.hardwareManager.rangeCheckAngle(headingError, -180, 180);

        return Range.clip(headingError, -1, 1);

    }

    public void moveRobot(double drive, double turn) {

        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        this.hardwareManager.individualPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }*/

