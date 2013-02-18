package com.questit.accompany2;

import android.app.Activity;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

public class AccompanyPreferences {

	protected String RosMasterIP;
	protected int images_rate;
	
	
	Activity owner;
	
	public AccompanyPreferences(
			final Activity activity) {
		
		this.owner = activity;
		
		this.RosMasterIP="192.168.1.109";
		this.images_rate=5;
	}
	
	public void loadPreferences() {
		final SharedPreferences prefs = owner.getSharedPreferences("accompany_gui_ros",owner.MODE_PRIVATE);
		
		this.RosMasterIP = prefs.getString("ros_master_ip",RosMasterIP);
		this.images_rate = prefs.getInt("images_rate", images_rate);
	}
	
	@Override
	public String toString() {
		return (this.RosMasterIP + " " +
				this.images_rate + " ");
	}
	
	public boolean equals(final AccompanyPreferences p) {
		if (this.RosMasterIP.equals(p.getRosMasterIP())) {
			if (this.images_rate==p.getImagesRate()) {
				return true;
			}
		}
		return false;
	}
	
	public String getRosMasterIP()
	{
		return RosMasterIP;
	}
	
	public int getImagesRate()
	{
		return images_rate;
	}
	
/*	public String getRobotIP()
	{
		return RobotIP;
	}
	
	public String getCameraIP()
	{
		return CameraIP;
	}
	
	public String getDataBaseIP()
	{
		return DataBaseIP;
	}
	
	public int getCameraPort()
	{
		return CameraPort;
	}
	
	public int getRobotPort()
	{
		return RobotPort;
	}
	
	public int getRSDPort()
	{
		return RobotStatusDetector_listening_port;
	}
	
	@Override
	public String toString() {
		return (this.RobotIP + " " +
				this.CameraIP + " " +
				this.DataBaseIP + " " +
				this.RobotPort + " " +
				this.CameraPort + " " + this.RobotStatusDetector_listening_port);
	}
	

	public boolean equals(final AccompanyPreferences p) {
		if (this.RobotIP.equals(p.getRobotIP())) {
			if (this.CameraIP.equals(p.getCameraIP())) {
				if (this.RobotPort == p.getRobotPort()) {
					if (this.CameraPort	== p.getCameraPort()) {
						if (this.RobotStatusDetector_listening_port == p.getRSDPort()) {
							if (this.DataBaseIP.equals(p.getDataBaseIP())) {
								return true;
							}
						}
					}
				}
			}
		}
		return false;
	}*/
	
}
