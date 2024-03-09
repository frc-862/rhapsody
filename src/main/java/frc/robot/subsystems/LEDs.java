package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.robot.Constants.RobotMap.PWM;

public class LEDs extends SubsystemBase {

	AddressableLED leds;
	AddressableLEDBuffer ledBuffer;
	Random random = new Random();
	private LED_STATES state = LED_STATES.DEFAULT;
	private Map<LED_STATES, Boolean> ledStates;

	public LEDs() {
		leds = new AddressableLED(PWM.LED_PORT);
		ledBuffer = new AddressableLEDBuffer(LEDsConstants.LED_LENGTH);

		leds.setLength(ledBuffer.getLength());
		leds.start();

		ledStates = new HashMap<LEDsConstants.LED_STATES, Boolean>();

		enableState(LED_STATES.START).withTimeout(7).schedule();

		for (LED_STATES i : Arrays.asList(LED_STATES.values())) {
			ledStates.put(i, false);
		}
	}

	@Override
	public void periodic() {
		switch (state) {
			case EMERGENCY:
				blink(-1, LEDsConstants.RED_HUE);
				break;

			case START:
				rainbow(-1);
				break;

			case COLLECTED:
				pulse(-1, LEDsConstants.GREEN_HUE);
				break;

			case SHOT:
				pulse(-1, LEDsConstants.GREEN_HUE);
				break;

			case FINISHED_CLIMB:
				pulse(-1, LEDsConstants.GREEN_HUE);
				break;

			case SHOOTING:
				blink(-1, LEDsConstants.YELLOW_HUE);
				break;

			case COLLECTING:
				pulse(-1, LEDsConstants.RED_HUE);
				break;

			case CHASING:
				pulse(-1, LEDsConstants.RED_HUE);
				break;

			case CLIMBING:
				blink(-1, LEDsConstants.PURPLE_HUE);
				break;

			case HAS_PIECE:
				setSolidStrandHSV(-1, LEDsConstants.ORANGE_HUE, 255, 255);
				break;

			case HAS_VISION:
				setSolidStrandHSV(-1, LEDsConstants.PINK_HUE, 255, 255);
				break;

			case DISABLED:
				setSolidHSV(0, 0, 0);
				break;

			case DEFAULT:
			 	swirl(-1);
				break;

			default:
				break;
		}

		if (DriverStation.isTest()) {
			if (ledStates.get(LED_STATES.COLLECTOR_BEAMBREAK)) {
				setStrandSingleHSV(0, 0, LEDsConstants.GREEN_HUE, 255, 255);
				setStrandSingleHSV(1, 0, LEDsConstants.GREEN_HUE, 255, 255);
			} else {
				setStrandSingleHSV(0, 0, LEDsConstants.RED_HUE, 255, 255);
				setStrandSingleHSV(1, 0, LEDsConstants.RED_HUE, 255, 255);
			}
			if (ledStates.get(LED_STATES.INDEXER_ENTER_BEAMBREAK)) {
				setStrandSingleHSV(0, 1, LEDsConstants.GREEN_HUE, 255, 255);
				setStrandSingleHSV(1, 1, LEDsConstants.GREEN_HUE, 255, 255);
			} else {
				setStrandSingleHSV(0, 1, LEDsConstants.RED_HUE, 255, 255);
				setStrandSingleHSV(1, 1, LEDsConstants.RED_HUE, 255, 255);
			}
			if (ledStates.get(LED_STATES.INDEXER_EXIT_BEAMBREAK)) {
				setStrandSingleHSV(0, 2, LEDsConstants.GREEN_HUE, 255, 255);
				setStrandSingleHSV(1, 2, LEDsConstants.GREEN_HUE, 255, 255);
			} else {
				setStrandSingleHSV(0, 2, LEDsConstants.RED_HUE, 255, 255);
				setStrandSingleHSV(1, 2, LEDsConstants.RED_HUE, 255, 255);
			}
			if (ledStates.get(LED_STATES.PIVOT_BOTTOM_SWITCH)) {
				setStrandSingleHSV(0, 3, LEDsConstants.GREEN_HUE, 255, 255);
				setStrandSingleHSV(1, 3, LEDsConstants.GREEN_HUE, 255, 255);
			} else {
				setStrandSingleHSV(0, 3, LEDsConstants.RED_HUE, 255, 255);
				setStrandSingleHSV(1, 3, LEDsConstants.RED_HUE, 255, 255);
			}
			if (ledStates.get(LED_STATES.PIVOT_TOP_SWITCH)) {
				setStrandSingleHSV(0, 4, LEDsConstants.GREEN_HUE, 255, 255);
				setStrandSingleHSV(1, 4, LEDsConstants.GREEN_HUE, 255, 255);
			} else {
				setStrandSingleHSV(0, 4, LEDsConstants.RED_HUE, 255, 255);
				setStrandSingleHSV(1, 4, LEDsConstants.RED_HUE, 255, 255);
			}
		}
		leds.setData(ledBuffer);
	}

	/**
	 * @param state the state to enable
	 * @return the command to enable the state
	 */
	public Command enableState(LED_STATES state) {
		return new StartEndCommand(() -> {
			ledStates.put(state, true);
			updateState();
		},
				() -> {
					ledStates.put(state, false);
					updateState();
				}).ignoringDisable(true);
	}

	public void updateState() {
		state = LED_STATES.DEFAULT;
		for (LED_STATES i : Arrays.asList(LED_STATES.values())) {
			Boolean value = ledStates.get(i);

			if (value != null && value && (i.getPriority() < state.getPriority())) {
				state = i;
			}
		}
	}

	/**
	 * @param strand the strand to rainbow
	 */
	public void rainbow(int strand) {
		for (int i = 0; i < LEDsConstants.STRAND_LENGTH.get(strand); i++) {
			setStrandSingleHSV(strand, i, (i + (int) (Timer.getFPGATimestamp() * 20)) % LEDsConstants.STRAND_LENGTH.get(strand) * 180 / LEDsConstants.STRAND_LENGTH.get(strand), 255,
					255);
		}
	}

	// public void flame(int strand) {
	// 	fireLength += (random.nextInt(3) - 1);
	// 	if (fireLength > LEDsConstants.STRAND_LENGTH.get(strand) || fireLength < 0) {
	// 		fireLength = LEDsConstants.STRAND_LENGTH.get(strand) / 2;
	// 	}
	// 	setSolidHSV(0, 0, 0);
	// 	for(int i = 0; i < Math.round(fireLength); i++) {
	// 		setStrandSingleHSV(i, LEDsConstants.RED_HUE, (int)(255 * Math.cos(Math.toRadians(i * 360 / LEDsConstants.STRAND_LENGTH.get(strand)) / 4)), 255);
	// 	}
	// }
	
	/**
	 * @param strand the strand to swirl
	 */
	public void swirl(int strand) {
		for (int i = 0; i < LEDsConstants.STRAND_LENGTH.get(strand); i++) {
			if (((i + (int) (Timer.getFPGATimestamp() * 10)) / LEDsConstants.SWIRL_SEGMENT_SIZE) % 2 == 0) {
				setStrandSingleHSV(strand, i, LEDsConstants.BLUE_HUE, 255, 255);
			} else {
				setStrandSingleHSV(strand, i, LEDsConstants.ORANGE_HUE, 255, 255);
			}
		}
	}

	/**
	 * @param strand the strand to blink
	 * @param hue the hue to blink
	 */
	public void blink(int strand, int hue) {
		if ((int) (Timer.getFPGATimestamp() * 10) % 2 == 0) {
			setSolidStrandHSV(strand, hue, 255, 255);
		} else {
			setSolidStrandHSV(strand, 0, 0, 0);
		}
	}

	/**
	 * @param strand the strand to pulse
	 * @param hue the hue to blink
	 */
	public void pulse(int strand, int hue) {
		setSolidStrandHSV(strand, hue, 255, (int) Math.abs((Math.sin(Timer.getFPGATimestamp() * 10) * 255)));
	}

	/**
	 * @param strand What strand to set
	 * @param index What LED to set
	 * @param h Hue
	 * @param s Saturation
	 * @param v Value
	 */
	public void setStrandSingleHSV(int strand, int index, int h, int s, int v) {
		ledBuffer.setHSV(index + LEDsConstants.STRAND_START.get(strand), h, s, v);
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
	 * @param strand What strand to set
	 * @param h Hue
	 * @param s Saturation
	 * @param v Value
	 */
	public void setSolidStrandHSV(int strand, int h, int s, int v) {
		for (int i = 0; i < LEDsConstants.STRAND_LENGTH.get(strand); i++) {
			setStrandSingleHSV(strand, i, h, s, v);
		}
	}

	/**
	 * @param h Hue
	 * @param s Saturation
	 * @param v Value
	 */
	public void setSolidHSV(int h, int s, int v) {
		for (var i = 0; i < LEDsConstants.LED_LENGTH; i++) {
			ledBuffer.setHSV(i, h, s, v);
		}
	}
}