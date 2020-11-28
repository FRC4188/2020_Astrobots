
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Parking3seconds", group="Linear Opmode")

public class Parking3seconds extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;
    private CRServo arm;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            if (runtime.seconds() < 3) {
                arm.setPower(0.3);
            }
            if (runtime.seconds() > 3 && runtime.seconds() < 4) {
                drivetrain (0, 1, 0);
            }
            if (runtime.seconds() > 4 && runtime.seconds() < 10){
                arm.setPower(0.3);
                drivetrain(1, 0, 0);
            }
            if (runtime.seconds() > 11 && runtime.seconds() < 14) {
                arm.setPower (-0.3);
            }
            if (runtime.seconds() > 14 && runtime.seconds() < 17){
                drivetrain(-1.0, 0, 0);
            }
            if (runtime.seconds() > 17 && runtime.seconds() < 18) {
                drivetrain (0, -1.0, 0);
            }
            if (runtime.seconds() > 18 && runtime.seconds() < 22) {
                shooterMotor.setPower(0.5);
            }
            if (runtime.seconds() > 22 && runtime.seconds() < 26) {
                shooterMotor.setPower(0.5);
                magazineMotor.setPower(1.0);
            }
            if (runtime.seconds() > 26 && runtime.seconds() < 26.5) {
                drivetrain(1.0, 0, 0);
            }
            if (runtime.seconds() > 26.5) {
                drivetrain(0, 0, 0);
            }
        }
    }

    private void initialize() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        arm = hardwareMap.get(CRServo.class, "wobbleArm");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime.reset();
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        frMotor.setPower(forward*1.05 - strafe - rotation);
        flMotor.setPower(forward/1.67 + strafe + rotation);
        brMotor.setPower(forward*1.05 + strafe - rotation);
        blMotor.setPower(forward/1.67 - strafe + rotation);
    }

}
