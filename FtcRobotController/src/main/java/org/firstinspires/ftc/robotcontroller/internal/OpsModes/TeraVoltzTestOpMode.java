/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.internal.OpsModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
@Disabled
public class TeraVoltzTestOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotTeravoltz robot           = new HardwarePushbotTeravoltz();   // Use a Pushbot's hardware
    String Version = "0.0.3";

    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() throws InterruptedException {
        double SHOOTER_INCREMENT = 0.01;
        double left;
        double right;
        double max;
        double KICKER_MAX_ANGLE = 0.5;
        double KICKER_MIN_ANGLE = 0;
        boolean rightbumper = false;
        boolean leftbumper = false;
        boolean collectorOn = false;
        boolean elevatorOn = false;
        double shooterPower = 0;
        int activeCount = 0;
        boolean lefttrigger;
        boolean righttrigger;
        int lastCyclerightBumper = 0;
        int lastCycleleftBumper = 0;
        boolean a;
        boolean b;
        boolean x;
        boolean y;
        boolean k;
        boolean c;
        boolean d;
        boolean stop;
        boolean open;
        boolean start;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "It is working and you loaded the package.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            activeCount ++;

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left  = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            righttrigger = 0 < gamepad2.right_trigger;
            lefttrigger = 0 < gamepad2.left_trigger;
            a = gamepad2.a;
            b = gamepad2.b;
            x = gamepad1.x;
            y = gamepad1.y;
            k = gamepad2.x;
            c = gamepad1.a;
            d = gamepad1.b;
            leftbumper = gamepad2.left_bumper;
            rightbumper = gamepad2.right_bumper;
            open = gamepad1.right_bumper;
            stop = gamepad1.left_bumper;
/*
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }
*/
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);

            if (rightbumper && lastCyclerightBumper > 10) {
                lastCyclerightBumper = 0;
                collectorOn = !collectorOn;
                if (collectorOn)
                    robot.ballCollector.setPower(1);
                else
                    robot.ballCollector.setPower(0);
            }
            else if (righttrigger) {
                robot.ballCollector.setPower(-1);
                collectorOn = true;
            }
            else {
                lastCyclerightBumper++;
            }

            if (leftbumper && lastCycleleftBumper > 10) {
                lastCycleleftBumper = 0;
                elevatorOn = !elevatorOn;
                if (elevatorOn)
                    robot.elevator.setPower(1);
                else
                    robot.elevator.setPower(0);
            }
            else if (lefttrigger) {
                robot.elevator.setPower(-1);
                elevatorOn = true;
            }
            else {
                lastCycleleftBumper++;
            }

            // a is pressed and shooter power less than 100%
            // add 20% to shooter power
            if (a) {
                shooterPower = shooterPower + SHOOTER_INCREMENT;
                if (shooterPower > 1)
                    shooterPower = 1;
                robot.rightShooter.setPower(shooterPower);
                robot.leftShooter.setPower(shooterPower);
            }
            // b is pressed and shooter power more than 0%
            // subtract 20% from shooter power
            if (b) {
                shooterPower = shooterPower - SHOOTER_INCREMENT;
                if (shooterPower < 0)
                    shooterPower = 0;
                robot.rightShooter.setPower(shooterPower);
                robot.leftShooter.setPower(shooterPower);
            }

            if(x) {
                robot.capBallLifter.setPower(1);
                telemetry.addData("Cap ball lifter:", "Extending");
            }

            // Cap ball lifter
            else if (y) {
                robot.capBallLifter.setPower(-1);
                telemetry.addData("Cap ball lifter:", "Lowering");
            }

            else {
                robot.capBallLifter.setPower(0);
                telemetry.addData("Cap ball lifter:", "Stopped");
            }


            //x is pressed and shooter extends or contracts

            if (open) {
                double f = robot.stopper.getPosition();
                if (f > 0) {
                    robot.stopper.setPosition(f - 0.05);
                }
                    telemetry.addData("Catch = ", f);
            }

            if (stop) {
                double f = robot.stopper.getPosition();
                if (f < 1) {
                    robot.stopper.setPosition(f + 0.05);
                }
                telemetry.addData("Catch = ", f);
            }

            //
            if(k) {
                robot.kicker.setPosition(KICKER_MAX_ANGLE);
                telemetry.addData("Kicker", "Raised");
            }
            else{
                robot.kicker.setPosition(KICKER_MIN_ANGLE);
                //telemetry.addData("Kicker", "Lowered");
            }

            //Button arm
            if (c) {
                robot.arm.setPower(1);
                telemetry.addData("Arm", "Extending");
            }
            else if (d) {
                robot.arm.setPower(-1);
                telemetry.addData("Arm", "Retracting");
            }
            else {
                robot.arm.setPower(0);
                telemetry.addData("Arm", "Stopped");
            }


            // Send telemetry message to signify robot running and provide infor;
            telemetry.addData("Version: ", "%s, activeCount: %d", this.Version, activeCount);
            telemetry.addData("left_stick",  "%.2f", left);
            telemetry.addData("right_stick", "%.2f", right);
            telemetry.addData("shooterPower", "%.2f", shooterPower);
            telemetry.addData("Catch", "%.2f");
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
