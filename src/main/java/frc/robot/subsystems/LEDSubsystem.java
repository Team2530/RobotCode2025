package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    public LEDSubsystem() {
        led = new AddressableLED(8);
        ledBuffer = new AddressableLEDBuffer(40);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }
    @Override
    public void periodic() {
        rainbow((int) (Timer.getFPGATimestamp()));
    }

    /* Periodic Funcitons */
    void rainbow(int tick) {
        for (int i = 0; i < ledBuffer.getLength(); ++i) {
            int hue = (tick + i * 180 / ledBuffer.getLength()) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
    }
    boolean toFlash(int tick, int period) {
        int flash = tick % period;
        return flash < period/2;
    }
    boolean toFlash(double hz) {
        double period = 1.0 / hz;
        double v = ((Timer.getFPGATimestamp()) % period) / period;
        return v < 0.5;
    }

    void sineColor(int r, int g, int b, double waves, double center, double amp, double tscroll) {
        double lambda = ledBuffer.getLength() / (2 * waves);
        for (int i = 0; i < 20; ++i) {
            double fx = Math.cos((i / lambda) * Math.PI + Timer.getFPGATimestamp() * tscroll * Math.PI) * amp + center;
            setRGB(i,
                    Math.min((int) (r * fx), 255),
                    Math.min((int) (g * fx), 255),
                    Math.min((int) (b * fx), 255));
        }
    }

    /* Quick */
    void setAllSolid(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); ++i)
            setRGB(i, r, g, b);
    }
    void setAllSolid(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); ++i)
            setColor(i, color);
    }

    /* Shortened Names */
    public void setRGB(int n, int r, int g, int b) {
        ledBuffer.setRGB(n, r, g, b);}
    public void setColor(int n, Color color) {
        ledBuffer.setRGB(n, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));}
    public void setHSV(int n, int h, int s, int v) {
        ledBuffer.setHSV(n, h, s, v);}
    public void push() {
        led.setData(ledBuffer);}
    public void off() {setAllSolid(0, 0, 0);}
}
