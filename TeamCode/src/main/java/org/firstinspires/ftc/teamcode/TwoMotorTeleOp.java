package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Arcade + MAX Shooter + Smooth", group = "Drive")
public class TwoMotorTeleOp extends LinearOpMode {

    private static final String LEFT_MOTOR_NAME   = "driveL";
    private static final String RIGHT_MOTOR_NAME  = "driveR";
    private static final String SHOOTY_MOTOR_NAME = "shooty";
    private static final String LEFT_CR_NAME      = "shootSL";
    private static final String RIGHT_CR_NAME     = "shootSR";

    private static final double DEADZONE       = 0.06;
    private static final double NORMAL_SCALE   = 0.85;
    private static final double SLOW_SCALE     = 0.45;
    private static final double TURBO_SCALE    = 1.00;

    private static final double EXPO_POWER     = 1.5;
    private static final double MAX_SLEW_PER_S = 2.5;

    private static final double STEP           = 0.05;
    private static final double FEED_FULL      = 1.0;

    // Voltage compensation target (nominal 12V battery)
    private static final double NOMINAL_VOLTAGE = 12.0;

    private DcMotor left, right;
    private DcMotorEx shooty;
    private CRServo crL, crR;
    private VoltageSensor battery;

    private double lastLeftCmd  = 0.0;
    private double lastRightCmd = 0.0;
    private double lastTimeS    = 0.0;
    private double currentFeedPower = 0.0;

    private boolean shooterHold = false; // X toggles pre-spin; A cancels

    @Override
    public void runOpMode() {
        left   = hardwareMap.get(DcMotor.class, LEFT_MOTOR_NAME);
        right  = hardwareMap.get(DcMotor.class, RIGHT_MOTOR_NAME);
        shooty = hardwareMap.get(DcMotorEx.class, SHOOTY_MOTOR_NAME);
        crL    = hardwareMap.get(CRServo.class, LEFT_CR_NAME);
        crR    = hardwareMap.get(CRServo.class, RIGHT_CR_NAME);

        // Pick any available voltage sensor (Control Hub / Expansion Hub)
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // For absolute max RPM, run the shooter open-loop WITHOUT encoder limiting
        shooty.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        crL.setDirection(CRServo.Direction.FORWARD);
        crR.setDirection(CRServo.Direction.FORWARD);

        setBothCR(0.0);
        shooty.setPower(0.0);

        telemetry.addLine("Drive: RightY=forward, LeftX=turn");
        telemetry.addLine("LB=Slow, RB=Turbo | RT=Shooter MAX | LT=Feed CR | X=Hold shooter ON | A=Stop shooter");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        lastTimeS = getRuntime();

        while (opModeIsActive()) {
            final double nowS = getRuntime();
            final double dt   = Math.max(1e-3, nowS - lastTimeS);
            lastTimeS = nowS;

            // -------- DRIVE --------
            double fwd = -gamepad1.right_stick_y;
            double rot =  gamepad1.left_stick_x;

            fwd = applyDeadzone(fwd, DEADZONE);
            rot = applyDeadzone(rot, DEADZONE);

            fwd = shapeExpo(fwd, EXPO_POWER);
            rot = shapeExpo(rot, EXPO_POWER);

            double scale = NORMAL_SCALE;
            if (gamepad1.left_bumper)  scale = SLOW_SCALE;
            if (gamepad1.right_bumper) scale = TURBO_SCALE;

            double targetLeft  = Range.clip((fwd + rot) * scale, -1.0, 1.0);
            double targetRight = Range.clip((fwd - rot) * scale, -1.0, 1.0);

            double maxStep = MAX_SLEW_PER_S * dt;
            double cmdLeft  = slewTo(lastLeftCmd, targetLeft,  maxStep);
            double cmdRight = slewTo(lastRightCmd, targetRight, maxStep);

            left.setPower(cmdLeft);
            right.setPower(cmdRight);

            lastLeftCmd  = cmdLeft;
            lastRightCmd = cmdRight;

            // -------- SHOOTER (MAX RPM) --------
            // X holds shooter ON; A cancels hold
            if (gamepad1.x) shooterHold = true;
            if (gamepad1.a) shooterHold = false;

            boolean triggerRequestsShooter = gamepad1.right_trigger > 0.05;
            boolean shooterOn = shooterHold || triggerRequestsShooter;

            double shootyCmd = shooterOn ? 1.0 : 0.0;

            // Voltage compensation to keep output as “hot” as possible when the battery sags
            double v = (battery != null) ? battery.getVoltage() : NOMINAL_VOLTAGE;
            if (v < 1.0) v = NOMINAL_VOLTAGE; // safety fallback
            double compensated = shootyCmd * (NOMINAL_VOLTAGE / v);
            compensated = Range.clip(compensated, 0.0, 1.0);

            shooty.setPower(compensated);

            // -------- FEED (CR SERVOS) --------
            double lt = gamepad1.left_trigger;
            if (lt > 0.05) {
                currentFeedPower = lt;            // proportional feed forward
                setBothCR(currentFeedPower);
            } else {
                if (gamepad1.dpad_up) {
                    currentFeedPower = Range.clip(currentFeedPower + STEP, -1.0, 1.0);
                    setBothCR(currentFeedPower);
                } else if (gamepad1.dpad_down) {
                    currentFeedPower = Range.clip(currentFeedPower - STEP, -1.0, 1.0);
                    setBothCR(currentFeedPower);
                }
                if (gamepad1.b) { currentFeedPower = FEED_FULL; setBothCR(currentFeedPower); }
                if (gamepad1.y) { currentFeedPower = 0.0;       setBothCR(currentFeedPower); }
            }

            // -------- TELEMETRY --------
            telemetry.addData("Drive",  "L:%.2f R:%.2f (scale %.2f)", cmdLeft, cmdRight, scale);
            telemetry.addData("Shooter", "on:%s rawCmd:%.2f batt:%.2fV out:%.2f",
                    shooterOn ? "Y" : "N", shootyCmd, v, compensated);
            telemetry.addData("FeedCR", "%.2f (LT/Dpad/B=full,Y=stop)", currentFeedPower);
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
        shooty.setPower(0);
        setBothCR(0.0);
    }

    private static double applyDeadzone(double v, double dz) {
        return (Math.abs(v) < dz) ? 0.0 : v;
    }

    private static double shapeExpo(double x, double p) {
        if (p <= 0) return x;
        double sign = Math.signum(x);
        double ax = Math.abs(x);
        double shaped = Math.pow(ax, p);
        return sign * shaped;
    }

    private static double slewTo(double current, double target, double maxStep) {
        double delta = target - current;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    }

    private void setBothCR(double power) {
        crL.setPower(power);
        crR.setPower(-power); // flip one if your roller directions need to oppose
    }
}
