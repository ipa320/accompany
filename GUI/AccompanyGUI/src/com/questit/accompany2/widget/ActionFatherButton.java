package com.questit.accompany2.widget;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.Socket;
import java.util.ArrayList;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.HttpClient;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;
import org.apache.http.params.BasicHttpParams;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.questit.accompany2.R;
import com.questit.accompany2.activities.ActionsListView;
import com.questit.accompany2.data.Action;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.FrameLayout.LayoutParams;

public class ActionFatherButton extends FrameLayout
{
	protected final int HIGHLIGHTED = Color.parseColor("#33AABB");
	protected final int TEXTSIZE=20;
	
	protected String text;
	protected int id;
	
	protected ActionsListView act;
	protected TextView tv;
	protected View.OnClickListener myListener;
	
	protected ArrayList<Action> sons;
	
	protected ActionFatherButton me;
	
	public ActionFatherButton(Context context,String text,ActionsListView ac,
			int id) {
		super(context);
		tv= new TextView(context);
		tv.setTextColor(Color.WHITE);
		tv.setTextSize(TEXTSIZE);
		tv.setPadding(2,4,2,4);
		this.setBackgroundColor(Color.TRANSPARENT);
		
		this.text=text;
		this.act=ac;
		tv.setText(text);
		
		
		this.setClickable(true);
		tv.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT));
		this.addView(tv);
		
		this.id=id;		
		me=this;
		
		myListener= new View.OnClickListener() {
			
			/*@SuppressLint("NewApi")
			@TargetApi(11)
			@Override
			public void onClick(View v) {
				sons= new ArrayList<Action>();
				
				//query to get the sons
				InputStream is=null;
			    String result="";
			    //data to send
			    ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();
			    nameValuePairs.add(new BasicNameValuePair("id",Integer.toString(me.id)));
			    Log.i("marco",""+me.id);
			    //attempt to connect to remote mysql database trought the apposite php web page
			    try{
			    	//setting timetouts for the connection!!
			    	HttpParams httpParameters = new BasicHttpParams();
			    	int timeoutConnection = 3000;  //the proper connection timeout
			    	HttpConnectionParams.setConnectionTimeout(httpParameters, timeoutConnection);
			    	int timeoutSocket = 5000;      //the operation timeout
			    	HttpConnectionParams.setSoTimeout(httpParameters, timeoutSocket);
			    	//the client with these parameters:
			    	HttpClient hc= new DefaultHttpClient(httpParameters); 
			    	HttpPost hp= new HttpPost("http://"+robotIp+"/getSons.php");
			    	//HttpPost hp= new HttpPost("http://192.168.1.104/php_labels.php");
			    	hp.setEntity(new UrlEncodedFormEntity(nameValuePairs));
		    		Log.i("INFO","Starting query");
		    		HttpResponse response = hc.execute(hp);
		    		Log.i("INFO","Executed the query");
		    		HttpEntity entity = response.getEntity();

		    		is = entity.getContent();
			    }
			    catch(Exception e){
			    	Log.e("ERROR", "Error in http connection to remote mysql db: "+ e.toString());
			    }
			    //convert response to string
			    try{

			    	BufferedReader reader = new BufferedReader(new InputStreamReader(is,"iso-8859-1"),8);
			    	StringBuilder sb = new StringBuilder();
			    	String line = null;
			    	while ((line = reader.readLine()) != null) {
			    		sb.append(line + "\n");
			    	}
			    	is.close();

			    	result=sb.toString();
			    }catch(Exception e){
			    	Log.e("ERROR", "Error converting result of http request: "+e.toString());
			    }
			    //parse json data
			    try{
			    	JSONArray jArray = new JSONArray(result);
			    	for(int i=0;i<jArray.length();i++){
			    		JSONObject json_data = jArray.getJSONObject(i);
		            
			    		sons.add(new Action(json_data.getString("ap_label"),
			    				json_data.getString("command"),json_data.getString("phraseal_feedback")));
		            
			    	}
			    }catch(JSONException e){
			    	Log.e("log_tag", "Error parsing data "+e.toString());
			    }
			    
			    //show Action dialog
			    LayoutInflater layoutInflater 
			     = (LayoutInflater)act.getBaseContext()
			      .getSystemService(act.LAYOUT_INFLATER_SERVICE);  
			    LinearLayout popupView =(LinearLayout)layoutInflater.inflate(R.layout.mypopupmenu, null);
			    MyPopupMenu popupWindow = new MyPopupMenu(act.getApplicationContext(),
			               popupView, 
			               LayoutParams.WRAP_CONTENT,  
			                     LayoutParams.WRAP_CONTENT,sons,
			                     getResources().getDrawable(R.drawable.transparent),robotIp,
			                     robot_port,act,me); 
			    me.measure(act.getWindowManager().getDefaultDisplay().getWidth(),
			    		act.getWindowManager().getDefaultDisplay().getHeight());
			    popupWindow.showAsDropDown(v,me.getMeasuredWidth()+100,-me.getMeasuredHeight());
			    
			    tv.setTextColor(HIGHLIGHTED);
			    me.setClickable(false);
				
			}*/
			@Override
			public void onClick(View v) {
				sons= new ArrayList<Action>();
				act.sendSonRequest(me.id, me);
			}
			
		};
		this.setOnClickListener(myListener);
		
	}
	
	public void setSons(String res)
	{
		//parse json data
	    try{
	    	JSONArray jArray = new JSONArray(res);
	    	for(int i=0;i<jArray.length();i++){
	    		JSONObject json_data = jArray.getJSONObject(i);
            
	    		sons.add(new Action(json_data.getString("ap_label"),
	    				json_data.getString("command"),json_data.getString("phraseal_feedback")));
            
	    	}
	    }catch(JSONException e){
	    	Log.e("log_tag", "Error parsing data "+e.toString());
	    }
	    
	    //show Action dialog
	    LayoutInflater layoutInflater 
	     = (LayoutInflater)act.getBaseContext()
	      .getSystemService(act.LAYOUT_INFLATER_SERVICE);  
	    LinearLayout popupView =(LinearLayout)layoutInflater.inflate(R.layout.mypopupmenu, null);
	    MyPopupMenu popupWindow = new MyPopupMenu(act.getApplicationContext(),
	               popupView, 
	               LayoutParams.WRAP_CONTENT,  
	                     LayoutParams.WRAP_CONTENT,sons,
	                     getResources().getDrawable(R.drawable.transparent),act,me); 
	    me.measure(act.getWindowManager().getDefaultDisplay().getWidth(),
	    		act.getWindowManager().getDefaultDisplay().getHeight());
	    popupWindow.showAsDropDown(me,me.getMeasuredWidth()+100,-me.getMeasuredHeight());
	    
	    tv.setTextColor(HIGHLIGHTED);
	    me.setClickable(false);
	}
	
	public String getCommand(String name)
	{
		for (int i=0;i<sons.size();i++)
			if (sons.get(i).label.equals(name)) return sons.get(i).command;
		
		return null;
	}
	
	public void reset()
	{
		sons.clear();
		tv.setTextColor(Color.WHITE);
		this.setClickable(true);
	}
	

}