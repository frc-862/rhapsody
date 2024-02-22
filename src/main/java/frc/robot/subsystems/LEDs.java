package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;
import frc.robot.Constants.RobotMap.PWM;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.LEDsConstants.LED_STATES;

public class LEDs extends SubsystemBase {

	AddressableLED leds;
	AddressableLEDBuffer ledBuffer;
	private LED_STATES state;
	private Map<LED_STATES, Boolean> ledStates;

	public LEDs() {
		leds = new AddressableLED(PWM.LED_PORT_1);
		ledBuffer = new AddressableLEDBuffer(LEDsConstants.LED_LENGTH);

		leds.setLength(ledBuffer.getLength());
		leds.start();

		ledStates = new HashMap<LEDsConstants.LED_STATES, Boolean>();
	}

	@Override
	public void periodic() {
		state = LED_STATES.DEFAULT;
		for (LED_STATES i : Arrays.asList(LED_STATES.values())) {
			Boolean value = ledStates.get(i);

			if (value != null && value && (i.getPriority() < state.getPriority())) {
				state = i;
			}
		}

		switch (state) {
			case EMERGENCY:
				blink(LEDsConstants.RED_HUE);
				break;

			case COLLECTED:
				pulse(LEDsConstants.GREEN_HUE);
				break;

			case SHOT:
				pulse(LEDsConstants.GREEN_HUE);
				break;

			case FINISHED_CLIMB:
				pulse(LEDsConstants.GREEN_HUE);
				break;

			case SHOOTING:
				blink(LEDsConstants.YELLOW_HUE);
				break;

			case COLLECTING:
				pulse(LEDsConstants.RED_HUE);
				break;

            case CHASING:
				pulse(LEDsConstants.RED_HUE);
				break;

			case CLIMBING:
				blink(LEDsConstants.PURPLE_HUE);
				break;

			case HAS_PIECE:
				setSolidHSV(LEDsConstants.ORANGE_HUE, 255, 255);
				break;

			case HAS_VISION:
				setSolidHSV(LEDsConstants.PINK_HUE, 255, 255);
				break;

			case DISABLED:
				setSolidHSV(0, 0, 0);
				break;

			default:
				swirl();
		}

		leds.setData(ledBuffer);

		LightningShuffleboard.setStringSupplier("LEDs", "State",() -> state.toString());
	}

	/**
	 * @param state the state to enable
	 * @return the command to enable the state
	 */
	public Command enableState(LED_STATES state) {
		return new StartEndCommand(() -> {
		},
				() -> {
				}).ignoringDisable(true);
	}

	public void rainbow() {
		for (int i = 0; i < LEDsConstants.LED_LENGTH; i++) {
			setSingleHSV(i, (i + (int) (Timer.getFPGATimestamp() * 20)) % ledBuffer.getLength() * 180 / 14, 255,
					100);
		}
	}

	public void swirl() {
		for (int i = 0; i < LEDsConstants.LED_LENGTH; i++) {
			if (((i + (int) (Timer.getFPGATimestamp() * 10)) / LEDsConstants.SWRIL_SEGMENT_SIZE) % 2 == 0) {
				setSingleHSV(i, LEDsConstants.BLUE_HUE, 255, 255);
			} else {
				setSingleHSV(i, LEDsConstants.ORANGE_HUE, 255, 255);
			}
		}
	}

	/**
	 * @param hue the hue to blink
	 */
	public void blink(int hue) {
		if ((int) (Timer.getFPGATimestamp() * 10) % 2 == 0) {
			setSolidHSV(hue, 255, 255);
		} else {
			setSolidHSV(0, 0, 0);
		}
	}

	/**
	 * @param hue the hue to blink
	 */
	public void pulse(int hue) {
		setSolidHSV(hue, 255, (int) Math.abs((Math.sin(Timer.getFPGATimestamp() * 10) * 255)));
	}

	/**
	 * @param index What LED to set
	 * @param h Hue
	 * @param s Saturation
	 * @param v Value
	 */
	public void setSingleHSV(int index, int h, int s, int v) {
		ledBuffer.setHSV(index, h, s, v);
	}

	/**
	 * @param h Hue
	 * @param s Saturation
	 * @param v Value
	 */
	public void setSolidHSV(int h, int s, int v) {
		for (var i = 0; i < LEDsConstants.LED_LENGTH; i++) {
			setSingleHSV(i, h, s, v);
		}
	}
}