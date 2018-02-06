package org.usfirst.frc.team3373.robot;

//import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LidarSensor {
	private I2C lidar;
	private int pdist = 0;
	//private int dist;
	//private byte dish;
	//private byte disl;
	private byte[] check = {00000000, 00000000};
	private boolean reqRead = false;
	private byte[] sendData = {00000000};
	//private byte[] distance = {00000000, 00000000};

	public LidarSensor() {
		SmartDashboard.putString("Connecting", "LIDAR Connecting");
		lidar = new I2C(I2C.Port.kOnboard, 0x62);
		lidar.write(0x02, 0x1d);
		lidar.write(0x04, 0x08);
		lidar.write(0x1c, 0x00);
	}
	
	public int getDistance() {
		if(!reqRead) {
			lidar.write(0x00, 0x00);
			lidar.write(0x00, 0x04);
			reqRead = true;
		}
		if (!busy()) {
			reqRead = false;
			return readDist();
		}
		return pdist;
	}
	
	private int readDist() {
		int dist = -1;
		byte[] distance = {00000000, 00000000};
		sendData[0] = (byte) 0x8f;
		lidar.writeBulk(sendData, 1);
		lidar.readOnly(distance, 2);
		SmartDashboard.putRaw("RawDistance", distance);
		//dist = convert(distance);
		dist = (distance[0]<<8)+distance[1];
		if(dist!=0) {
			pdist = dist;
			return dist;
		}
		return pdist;
	}
	
	/*private int convert(byte[] distance) {
		int cdist = 0;
		for(int i = 1; i < 9; i++) {
			int l = i - 1;
			cdist += Math.pow(distance[0] << l, i);
		}
		for(int i = 8; i < 17; i++) {
			int l = i - 1;
			cdist += Math.pow(distance[1] << l, i);
		}
		return cdist;
	}*/

	private boolean busy() {
		sendData[0] = 0x01;
		lidar.writeBulk(sendData, 1);
		lidar.readOnly(check, 2);
		System.out.println("Check: " + check);
		if (bitRead(check[0], 0)) {
			return true;
		} else {
			return false;
		}
	}
	
	private boolean bitRead(byte b, int bit) {
		return (b & (1 << bit)) != 0;
	}
}
