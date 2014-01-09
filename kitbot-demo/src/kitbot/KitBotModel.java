package kitbot;

import jssc.SerialPort;
import jssc.SerialPortException;

public class KitBotModel {
	private SerialPort serialPort;
    private byte motorA = 0;
    private byte motorB = 0;
    
	public KitBotModel() {
		try {
			serialPort = new SerialPort("COM3");
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
        }
        catch (Exception ex){
            System.out.println(ex);
        }
	}
	//oldPower are historic power of the motor.
	private double oldPowerA =0;
	private double oldPowerB =0;
	public void setMotors( double powerA, double powerB ) {
		double maxAcc = 0.01;
		if(Math.abs(powerA - oldPowerA)>maxAcc){
			double temp = powerA;
			powerA = oldPowerA + maxAcc*Math.signum(powerA-oldPowerA);
			oldPowerA = powerA;
		}
		if(Math.abs(powerB - oldPowerB)>maxAcc){
			double temp = powerB;
			powerB = oldPowerB + maxAcc*Math.signum(powerB-oldPowerB);
			oldPowerB = powerB;
		}
		motorA = (byte)(-powerA*127);
		motorB = (byte)(powerB*127);
		modified();
	}
	private double smoothMotor(double oldPower){
		double newPower = oldPower;
		int sign = (int)(newPower/Math.abs(newPower));
		if (sign*newPower< 0.08){
			if (sign*newPower >0.03){
				newPower = sign*0.03;
			}else{
				newPower = 0.0;
			}
		}
		else if(sign*newPower>0.8){
			newPower = sign*0.8;
		}
		return newPower;
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
	
	public void finalize() {
		try {
			serialPort.closePort();
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
}
