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
	protected int cob_version;
	
	protected int ap_update;
	protected int exp_update;
	
	protected String last_user;
	
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
		this.cob_version=AccompanyGUIApp.COB36;
		this.last_user="M";
		this.ap_update=1;
		this.exp_update=1;
	}
	
	public void loadPreferences() {
		final SharedPreferences prefs = owner.getSharedPreferences("accompany_gui_ros",owner.MODE_PRIVATE);
		
		this.RosMasterIP = prefs.getString("ros_master_ip",RosMasterIP);
		this.images_rate = prefs.getInt("images_rate", images_rate);
		this.speechMode = prefs.getBoolean("speech_mode", speechMode);
		//this.database_ip = prefs.getString("database_ip",database_ip);
		this.database_url=prefs.getString("database_url",database_url);
		this.status_url=prefs.getString("status_url", status_url);
		this.cob_version=prefs.getInt("cob_version", AccompanyGUIApp.COB36);
		this.last_user=prefs.getString("last_logged_user",last_user);
		this.ap_update=prefs.getInt("actionpossibilities_update_frequency", ap_update);
		this.exp_update=prefs.getInt("expression_update_frequency", exp_update);
	}
	
	@Override
	public String toString() {
		return (this.RosMasterIP + " " +
				this.images_rate + " "+ this.speechMode+" "+this.cob_version+" "+this.database_url+ " "+this.status_url+ " " +
				this.last_user);
	}
	
	public boolean equals(final AccompanyPreferences p) {
		if (this.RosMasterIP.equals(p.getRosMasterIP())) {
			if (this.images_rate==p.getImagesRate()) {
				if (this.speechMode==p.getSpeechMode()){
					if (this.cob_version==p.getCobVersion()){
						if (this.database_url.equals(p.getDatabaseUrl())){
							if (this.status_url.equals(p.getRobotStatusControlUrl())){
								if (this.last_user.equals(p.getLastUser())) {
									if (this.ap_update==p.getApUpdateFrequency()){
										if (this.exp_update==p.getExpressionUpdateFrequency()){
											return true;
										}
									}
								}
							}
						}
					}
				}
			}
		}
		return false;
	}
		
	public int getExpressionUpdateFrequency()
	{
		return exp_update;
	}
	
	public int getApUpdateFrequency()
	{
		return ap_update;
	}
	
	public int getCobVersion()
	{
		return cob_version;	
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
	
	public String getLastUser()
	{
		return last_user;
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
