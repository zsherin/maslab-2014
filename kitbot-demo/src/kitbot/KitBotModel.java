package kitbot;

import jssc.SerialPort;
import jssc.SerialPortException;

public class KitBotModel {
	private SerialPort serialPort;
    private byte motorA = 0;
    private byte motorB = 0;
    private double x = 0;
    private double y = 0;
	public KitBotModel() {
		try {
			serialPort = new SerialPort("COM6");
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
        }
        catch (Exception ex){
            System.out.println(ex);
        }
	}
	//oldPower are historic power of the motor.
	public void setMotors( double powerA, double powerB ) {
    	motorA = (byte)(-motorConstrain(powerA)*127);
		motorB = (byte)(motorConstrain(powerB)*127);
		modified();
	}
	public double motorConstrain(double oldpower){
		if(oldpower > 1)
			return 1;
		else if (oldpower <-1)
			return -1;
		return oldpower;
	}
	public void modified() {
		try {
			byte[] data = new byte[4];
			data[0] = 'S';		// Start signal "S"
			data[1] = motorA;	// Motor A data
			data[2] = motorB;	// Motor B data
			data[3] = 'E';		// End signal "E"
			serialPort.writeBytes(data);
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
	public void updateMotor(){
		try {
			byte[] data = new byte[4];
			data[0] = 'S';		// Start signal "S"
			data[1] = motorA;	// Motor A data
			data[2] = motorB;	// Motor B data
			data[3] = 'E';		// End signal "E"
			serialPort.writeBytes(data);
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
	
	public void finalize() {
		try {
			serialPort.closePort();
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
}
