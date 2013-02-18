package com.questit.accompany2.activities;

import com.questit.accompany2.AccompanyGUI_app2;
import com.questit.accompany2.R;

import android.app.Activity;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

public class Settings extends Activity {
	
	protected String RosMasterIP="192.168.1.109";
	protected int images_rate=5;
	
	private EditText editIP;
	private EditText editRate;
	private Button save;
	
	protected AccompanyGUI_app2 myApp;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
        setContentView(R.layout.settings);
        
        myApp=(AccompanyGUI_app2)this.getApplication();
        
        editIP=(EditText)this.findViewById(R.id.master_et);
        editRate=(EditText)this.findViewById(R.id.rate_et);
        save=(Button)this.findViewById(R.id.ok_btn_settings);
        
        readPreferences();
        
        Bundle b= getIntent().getExtras();
        
        if (b==null)
        {
        	editIP.setFocusable(false);
        	editRate.requestFocus();
        	save.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					RosMasterIP= editIP.getText().toString();     //robot
					images_rate= Integer.parseInt(editRate.getText().toString());
					setPreferences();
					
					setResult(Activity.RESULT_OK,null);
					finish();
				}});
        }
        else
        {
        	save.setOnClickListener(new View.OnClickListener(){
				@Override
				public void onClick(View v) {
					RosMasterIP= editIP.getText().toString();     //robot
					images_rate= Integer.parseInt(editRate.getText().toString());
					setPreferences();
					
					setResult(Activity.RESULT_OK,null);
					finish();
				}});
        }
	}
	

    //reading the current shared preferences:
    private void readPreferences() {
        SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
            MODE_PRIVATE);
        //reading the current shared preferences:
        RosMasterIP = preferences.getString("ros_master_ip", RosMasterIP);  //robot
        images_rate = preferences.getInt("images_rate", images_rate);

             
        //writing the current preferences on screen:
        editIP.setText(RosMasterIP);
        editRate.setText(Integer.toString(images_rate));
            
     }
         
    private void setPreferences() {
	    SharedPreferences preferences = getSharedPreferences("accompany_gui_ros",
	        MODE_PRIVATE);
	    SharedPreferences.Editor editor = preferences.edit();

	    editor.putString("ros_master_ip",RosMasterIP);
	    editor.putInt("images_rate", images_rate);
	   
	    editor.commit();
	  }

	/*
	private String database_ip="192.168.1.110";
	private String ServerIP_rob="192.168.1.110";  //robot values
	private int Port_rob=5000;
	private String ServerIP_cam="192.168.1.110";  //camera values
	private int Port_cam=55555;
	private int rsd_port=5001;                      //the port where the robot status detector
											   //shoul be listening.
	
	//layout stuffs
	private EditText editIP_rob;
	private EditText editPort_rob;
	private EditText editIP_cam;
	private EditText editPort_cam;
	private EditText rsd_port_et;
	private Button save;
	private EditText db_ip_et;
	
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
        setContentView(R.layout.settings);
		
        //binding layout objects to varibales here
        editIP_rob=(EditText)this.findViewById(R.id.editIP_robot);     //robot settings
        editPort_rob=(EditText)this.findViewById(R.id.editPort_robot);
        editIP_cam=(EditText)this.findViewById(R.id.editIP_cam);      //Remote camera settings
        editPort_cam=(EditText)this.findViewById(R.id.editPort_cam);
        rsd_port_et=(EditText)this.findViewById(R.id.editRSD_port);
        save=(Button)this.findViewById(R.id.settingsOkButton);
        db_ip_et=(EditText)this.findViewById(R.id.edit_db_ip);
    
        readPreferences();
	
        save.setOnClickListener(new Button.OnClickListener()
        {
			@Override
			public void onClick(View v) {
				//reading the calues from text edits: 
				ServerIP_rob= editIP_rob.getText().toString();     //robot
				Port_rob= Integer.parseInt(editPort_rob.getText().toString());
				ServerIP_cam= editIP_cam.getText().toString();     //camera
				Port_cam= Integer.parseInt(editPort_cam.getText().toString());
				rsd_port=Integer.parseInt(rsd_port_et.getText().toString());
				database_ip=db_ip_et.getText().toString();
				//saving the new values on the shared preferences
				setPreferences();
				
				setResult(Activity.RESULT_OK,null);
				finish();
				}	
		});
        
	}
	
	//method that reads the preferences and set them on the relatives edit boxes
	private void readPreferences() {
        SharedPreferences preferences = getSharedPreferences("accompany_gui",
            MODE_PRIVATE);
        //reading the current shared preferences:
        ServerIP_rob = preferences.getString("robot_ip", ServerIP_rob);  //robot
        Port_rob= preferences.getInt("robot_port", Port_rob);
        ServerIP_cam = preferences.getString("camera_ip", ServerIP_cam);  //camera
        Port_cam= preferences.getInt("camera_port", Port_cam);
        rsd_port=preferences.getInt("rsd_listening_port",rsd_port);
        database_ip= preferences.getString("database_ip", database_ip);
         
        //writing the current preferences on screen:
        editIP_rob.setText(ServerIP_rob);
        editPort_rob.setText(Integer.toString(Port_rob));
        editIP_cam.setText(ServerIP_cam);
        editPort_cam.setText(Integer.toString(Port_cam));
        rsd_port_et.setText(Integer.toString(rsd_port));
        db_ip_et.setText(database_ip);
        
      }
	
	//method that store the inserted preferences in the shared preferences of the Activities
	 private void setPreferences() {
		    SharedPreferences preferences = getSharedPreferences("accompany_gui",
		        MODE_PRIVATE);
		    SharedPreferences.Editor editor = preferences.edit();

		    editor.putString("robot_ip", ServerIP_rob);
		    editor.putInt("robot_port", Port_rob);
		    editor.putString("camera_ip", ServerIP_cam);
		    editor.putInt("camera_port", Port_cam);
		    editor.putInt("rsd_listening_port", rsd_port);
		    editor.putString("database_ip",database_ip);
		    editor.commit();
		  }
	*/

}