package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;
import frc.robot.Constants.LEDsConstants.LED_STATES;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class LEDs extends SubsystemBase {
	AddressableLED leds;
	AddressableLEDBuffer ledBuffer;
	private LED_STATES state = LED_STATES.OFF;
	private Map<LED_STATES, Boolean> ledStates;

	/** Creates a new  */
	public LEDs() {
		leds = new AddressableLED(LEDsConstants.LED_PWM_PORT);
		ledBuffer = new AddressableLEDBuffer(LEDsConstants.LED_BUFFER_TIME);
		leds.setLength(ledBuffer.getLength());
		leds.start();
		ledStates = new HashMap<LEDsConstants.LED_STATES, Boolean>();

	}

	@Override
	public void periodic() {
		state = LED_STATES.OFF;
		for (LED_STATES i : Arrays.asList(LED_STATES.values())) {
			Boolean value = ledStates.get(i);

			if (value != null && value && (i.getPriority() < state.getPriority())) {
				state = i;
			}
		}

		switch (state) {
			case HAS_POSE:
				rainbow();
				break;

            case COLLECTED:
				pulse(240);
				break;

            case SHOT:
				pulse(240);
				break;

            case FINISHED_CLIMB:
				pulse(240);
				break;

            case SHOOTING:
				blink(15);
				break;

            case CHASING:
				blink(0);
				break;

            case READYING_SHOOT:
				pulse(15);
				break;

            case CLIMBED:
				setSolidHSV(315, 255, 255);
				break;

            case CAN_SHOOT:
				setSolidHSV(15, 255, 255);
				break;

            case HAS_PIECE:
			    setSolidHSV(5, 255, 255);
				break;

            case HAS_VISION:
				setSolidHSV(355, 255, 255);
				break;

			case MIXER:
				LightningShuffleboard.set("LEDs","Mixer Hue", 0);
				setSolidHSV((int)LightningShuffleboard.getDouble("LEDs", "Mixer Hue", 0), 255, 255);
				break;

			case OFF:
				swirl(4);
				break;
			
			default:
				setSolidRGB(0, 0, 0);

		}

		leds.setData(ledBuffer);
	}

	/**
	 * @param segmentSize size of each color segment
	 */
	public void swirl(int segmentSize) {
		for (int i = 0; i < LEDsConstants.LED_BUFFER_TIME; i++) {
			if (((i + (int)(Timer.getFPGATimestamp() * 10)) / segmentSize) % 2 == 0) {
				setIndexRGB(i, 0, 0, 255);
			} else {
				setIndexRGB(i, 255, 35, 0);
			}
		}
	}

	/**
	 * @param hue the hue to blink
	 */
	public void blink(int hue) {
		if ((int)(Timer.getFPGATimestamp() * 10) % 2 == 0) {
			setSolidRGB(hue, 255, 255);
		} else {
			setSolidRGB(0, 0, 0);
		}
	}
	
	/**
	 * @param hue the hue to blink
	 */
	public void pulse(int hue) {
		setSolidHSV(hue, 255, (int) Math.abs((Math.sin(Timer.getFPGATimestamp() * 2) * 255)));
	}

	
	public void rainbow() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setHSV(i,(i + (int)(Timer.getFPGATimestamp() * 20)) % ledBuffer.getLength() * 180 / 14 , 255, 100);
		}
	}
	
	public void setState(LED_STATES state, boolean value) {
		ledStates.put(state, value);
	}

	public void disable() {
		if(ledStates.get(LED_STATES.DISABLED) != null && ledStates.get(LED_STATES.DISABLED)) {
			ledStates.put(LED_STATES.DISABLED, false);
		} else {
			ledStates.put(LED_STATES.DISABLED, true);
		}
	}

	public int getBufferLength() {
		return ledBuffer.getLength();
	}

	public void setSolidRGB(int r, int g, int b) {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, r, g, b);
		}
	}

	public void setIndexRGB(int index, int r, int g, int b) {
		ledBuffer.setRGB(index, r, g, b);
	}

	public void setSolidHSV(int h, int s, int v) {
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setHSV(i, h, s, v);
		}
	}

	public void setIndexHSV(int index, int h, int s, int v) {
		ledBuffer.setHSV(index, h, s, v);
	}
}