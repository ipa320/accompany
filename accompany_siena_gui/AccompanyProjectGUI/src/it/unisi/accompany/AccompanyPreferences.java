package it.unisi.accompany;

import android.app.Activity;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

public class AccompanyPreferences {

	protected String RosMasterIP;
	protected int images_rate;
	protected boolean speechMode;
	//protected String database_ip;
	protected String database_url;
	protected String status_url;
	
	Activity owner;
	
	public AccompanyPreferences(
			final Activity activity) {
		
		this.owner = activity;
		
		this.RosMasterIP="10.0.1.5";
		this.images_rate=5;
		this.speechMode=false;
		//this.database_ip="10.0.1.5";
		this.database_url="http://10.0.1.5:9995/";
		this.status_url="http://10.0.1.5:9996/";
	}
	
	public void loadPreferences() {
		final SharedPreferences prefs = owner.getSharedPreferences("accompany_gui_ros",owner.MODE_PRIVATE);
		
		this.RosMasterIP = prefs.getString("ros_master_ip",RosMasterIP);
		this.images_rate = prefs.getInt("images_rate", images_rate);
		this.speechMode = prefs.getBoolean("speech_mode", speechMode);
		//this.database_ip = prefs.getString("database_ip",database_ip);
		this.database_url=prefs.getString("database_url",database_url);
		this.status_url=prefs.getString("status_url", status_url);
	}
	
	@Override
	public String toString() {
		return (this.RosMasterIP + " " +
				this.images_rate + " "+ this.speechMode+" "+this.database_url+ " "+this.status_url);
	}
	
	public boolean equals(final AccompanyPreferences p) {
		if (this.RosMasterIP.equals(p.getRosMasterIP())) {
			if (this.images_rate==p.getImagesRate()) {
				if (this.speechMode==p.getSpeechMode()){
					//if (this.database_ip.equals(p.getDatabaseIp())){
						if (this.database_url.equals(p.getDatabaseUrl())){
							if (this.status_url.equals(p.getRobotStatusControlUrl())){
								return true;
							}
						}
					//}
				}
			}
		}
		return false;
	}
	
	public boolean getSpeechMode()
	{
		return speechMode;
	}
	
	public String getRosMasterIP()
	{
		return RosMasterIP;
	}
	
	public int getImagesRate()
	{
		return images_rate;
	}
	
	/*public String getDatabaseIp()
	{
		return database_ip;
	}*/
	
	public String getDatabaseUrl()
	{
		return database_url;
	}
	
	public String getRobotStatusControlUrl()
	{
		return status_url;
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
