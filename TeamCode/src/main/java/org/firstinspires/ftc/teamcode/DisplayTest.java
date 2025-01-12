package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DisplayTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AlphaDisplay display = hardwareMap.get(AlphaDisplay.class, "display");
        display.writeCharacter('A', 0, true);
        display.writeCharacter('B', 1, true);
        display.writeCharacter('C', 2, true);
        display.writeCharacter('D', 3, true);
        display.updateDisplay();
    }
}
