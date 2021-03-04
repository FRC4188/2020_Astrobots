
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

//2.286
@Autonomous(name = "ParkingTest")
public class ParkingTest extends LinearOpMode {

    private DcMotorEx frMotor, flMotor, brMotor, blMotor;
    private DcMotor intakeMotor, magazineMotor, verticalArm;
    private DcMotorEx shooterMotor;
    private CRServo horizontalArm;
    private DcMotorEx shooterMotorEx;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //
        waitForStart();

        horizontalArm.setPower(-1.0);

        drivetrain(1.4, 0, 0);
        setAllPower(1.0);

        while (opModeIsActive() && isRobotBusy()) {
            //checkMotors();
            idle();
        }
        setAllPower(0.0);

        resetStartTime();

        runtime.reset();

        double shooterTime = runtime.seconds();

        int shootingStage = 0;
        boolean isShooting = true;

        double timeShot = runtime.seconds();

        while (opModeIsActive()) {
            telemetry.addData("shooter-vel", shooterMotorEx.getVelocity());
            telemetry.update();

            double timeSinceShot = runtime.seconds() - timeShot;

            if (!isShooting && timeSinceShot < 0.5) {
                drivetrainSlow(0, -1, -0.03);
            } else {
                drivetrainSlow(0, 0, 0);
                isShooting = true;
            }

            if (isShooting) {
                shooterMotorEx.setVelocity(1150);
                if (shooterMotorEx.getVelocity() > 1120) {
                    if (runtime.seconds() - shooterTime > 2) {
                        shooterMotorEx.setVelocity(0);
                        magazineMotor.setPower(0);
                        intakeMotor.setPower(0);
                        isShooting = false;
                        shootingStage++;

                        if (shootingStage > 2)
                            break;

                        timeShot = runtime.seconds();
                        continue;
                    }
                    magazineMotor.setPower(1.0);
                    intakeMotor.setPower(-1.0);
                } else shooterTime = runtime.seconds();

            }


        }


    }

    private void initialize() {

        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");


        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalArm = hardwareMap.get(CRServo.class, "hArm");
        verticalArm = hardwareMap.get(DcMotor.class, "vArm");
        verticalArm.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotorEx = (DcMotorEx) shooterMotor;


    }

    private void setMotorMode(DcMotor.RunMode runMode) {
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }

    private void setAllPower(double power) {
        frMotor.setPower(power);
        flMotor.setPower(power);
        brMotor.setPower(power);
        blMotor.setPower(power);
    }

    private void checkMotors() {
        if (!frMotor.isBusy()) {
            frMotor.setPower(0);
        }

        if (!flMotor.isBusy()) {
            flMotor.setPower(0);
        }

        if (!brMotor.isBusy()) {
            brMotor.setPower(0);
        }

        if (!blMotor.isBusy()) {
            blMotor.setPower(0);
        }
    }

    private boolean isRobotBusy() {
        return blMotor.isBusy();
    }

    /**
     * Increment a value by delta and return the new value.
     *
     * @param forwardDistance  the distance, in meters, the robot should move forward
     * @param strafeDistance   the distance, in meters, the robot should strafe sideways
     * @param rotationDistance the angle to rotate the robot
     */
    private void drivetrain(double forwardDistance, double strafeDistance, double rotationDistance) {

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setTargetPosition(frMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance - strafeDistance - rotationDistance)));
        flMotor.setTargetPosition(flMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance + strafeDistance + rotationDistance)));
        brMotor.setTargetPosition(brMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance + strafeDistance - rotationDistance)));
        blMotor.setTargetPosition(blMotor.getCurrentPosition() + (int) Math.round(Constants.TICKS_PER_METER * (forwardDistance - strafeDistance + rotationDistance)));

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void drivetrainSlow(double forward, double strafe, double rotation) {
        double frPower = forward - strafe - rotation;
        double flPower = forward + strafe + rotation;
        double brPower = forward + strafe - rotation;
        double blPower = forward - strafe + rotation;

        // Divide all power by the greatest absolute value to maintain proportionality while not exceeding +/- 1
        frMotor.setVelocity(frPower * 500);
        flMotor.setVelocity(flPower * 500);
        brMotor.setVelocity(brPower * 500);
        blMotor.setVelocity(blPower * 500);
    }

    private void driveSeconds(double forward, double strafe, double rotation) {
        frMotor.setPower(forward - strafe - rotation);
        flMotor.setPower(forward + strafe + rotation);
        brMotor.setPower(forward + strafe - rotation);
        blMotor.setPower(forward - strafe + rotation);
    }

}
