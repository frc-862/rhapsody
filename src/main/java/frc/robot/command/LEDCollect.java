package frc.robot.command;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class LEDCollect extends Command {
	private double startTime;
	private LEDs leds;

	public LEDCollect(LEDs leds) {
		this.leds = leds;
		startTime = Timer.getFPGATimestamp();

		addRequirements(leds);
	}

	@Override
	public void initialize() {
		leds.setSolidRGB(255, 0, 0);
	}

	@Override
	public void execute() { 
	}

	@Override
	public void end(boolean interrupted) {
		leds.setSolidRGB(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - startTime > 1;
	}
}
