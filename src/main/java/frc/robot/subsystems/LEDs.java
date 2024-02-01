package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;

public class LEDs extends SubsystemBase {
	AddressableLED leds;
	AddressableLEDBuffer ledBuffer;

	/** Creates a new LEDs. */
	public LEDs() {
		leds = new AddressableLED(LEDsConstants.LED_PWM_PORT);
		ledBuffer = new AddressableLEDBuffer(LEDsConstants.LED_BUFFER_TIME);
		leds.setLength(ledBuffer.getLength());
		leds.start();
	}

	@Override
	public void periodic() {
		leds.setData(ledBuffer);
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