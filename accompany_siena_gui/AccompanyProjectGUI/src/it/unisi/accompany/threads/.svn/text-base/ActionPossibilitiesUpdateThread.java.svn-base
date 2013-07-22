package it.unisi.accompany.threads;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.HttpClient;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.client.utils.URLEncodedUtils;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;
import org.apache.http.params.BasicHttpParams;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import android.os.Handler;
import android.util.Log;
import it.unisi.accompany.AccompanyGUIApp;
import it.unisi.accompany.msgs_and_data.AP_couple;
import it.unisi.accompany.widget.ActionPossibilityWidget;
import it.unisi.accompany.widget.actionliststuffs.Action;

public class ActionPossibilitiesUpdateThread extends Thread{
	
	protected final String TAG="AccompanyGUI-ApUpdateThread";
	protected final int DEFAULT_SLEEPTIME=1000;
	
	protected AccompanyGUIApp myApp;
	protected boolean stop;
	protected String url;
	
	protected Handler myHandler;
	
	protected int sleeptime;
	protected ArrayList<AP_couple> user_aps,robot_aps,list_aps;
	
	protected String last_response;
	
	//CONSTRUCTOR -> DEFAULT SLEEPTIME
	public ActionPossibilitiesUpdateThread(AccompanyGUIApp ma,Handler h,String ip,String port)
	{
		super();
		this.myApp=ma;
		this.myHandler=h;
		user_aps= new ArrayList<AP_couple>();
		robot_aps= new ArrayList<AP_couple>();
		list_aps= new ArrayList<AP_couple>();
		
		this.sleeptime=DEFAULT_SLEEPTIME;
		this.stop=false;
		
		createBaseUrl(ip,port);
	}
	
	//constructor with sleeptime
	public ActionPossibilitiesUpdateThread(AccompanyGUIApp ma,Handler h,String ip,String port, int timesleep)
	{
		super();
		this.myApp=ma;
		this.myHandler=h;
		user_aps= new ArrayList<AP_couple>();
		robot_aps= new ArrayList<AP_couple>();
		list_aps= new ArrayList<AP_couple>();
		
		this.sleeptime=timesleep;
		this.stop=false;
		
		createBaseUrl(ip,port);
	}
	
	public void setSleepTime(int st)
	{
		Log.v(TAG,"Setting sleeptime: "+st+" (old "+sleeptime+")" );
		this.sleeptime=st;
	}
	
	//CONSTRUCTOR -> DEFAULT SLEEPTIME
	public ActionPossibilitiesUpdateThread(AccompanyGUIApp ma,Handler h,String uu)
	{
		super();
		this.myApp=ma;
		this.myHandler=h;
		user_aps= new ArrayList<AP_couple>();
		robot_aps= new ArrayList<AP_couple>();
		list_aps= new ArrayList<AP_couple>();
		
		this.stop=false;
		this.sleeptime=DEFAULT_SLEEPTIME;
		
		createBaseUrl(uu);
	}
	
	//CONSTRUCTOR WITH SLEEPTIME
	public ActionPossibilitiesUpdateThread(AccompanyGUIApp ma,Handler h,String uu, int st)
	{
		super();
		this.myApp=ma;
		this.myHandler=h;
		user_aps= new ArrayList<AP_couple>();
		robot_aps= new ArrayList<AP_couple>();
		list_aps= new ArrayList<AP_couple>();
		
		this.stop=false;
		this.sleeptime=st;
		
		createBaseUrl(uu);
	}
	
	public void createBaseUrl(String ip, String prt)
	{
		url="http://"+ip+":"+prt+"/";
	}
	
	public void createBaseUrl(String uu)
	{
		url=uu;
	}
	
	@Override
	public void run()
	{
		//wait a couple of second for the load everything
		try {
			Thread.sleep(1500);
		} catch (InterruptedException e) {
			Log.e(TAG,"Sleeping problem!");
			stop=true;
		}
		//reading first values
		user_aps= getUserAps();
		robot_aps= getRobotAps();
		list_aps= getFullList();
		
		while(!stop)
		{
			try {
				Thread.sleep(sleeptime);
			} catch (InterruptedException e) {
				Log.e(TAG,"Sleeping problem!");
			}
			// fai direttamente qua hhtpp client sincorno! e confronta con valori vecchi.
			uncheckAll();
			ArrayList<AP_couple> news= filterAps(getUserAps());
			//control of user aps
			for (int i=0;i<news.size();i++)
				for (int j=0;j<user_aps.size();j++)
					if(user_aps.get(j).compare(news.get(i)))
					{
						user_aps.get(j).check=true;
						news.get(i).check=true;
					}
			//update of user aps
			for (int i=0;i<user_aps.size();i++)
				if (user_aps.get(i).check!=true) 
				{
					final int idf=user_aps.get(i).getId();
					myHandler.post(new Runnable(){
						@Override
						public void run() {
							myApp.removeUserAp(idf);
						}});
				}
			for (int i=0;i<news.size();i++)
				if (news.get(i).check!=true) 
				{
					final int idf=news.get(i).getId();
					myHandler.post(new Runnable(){
						@Override
						public void run() {
								myApp.addUserAp(idf,last_response);
						}});
				}
			user_aps=news;
			
			//ROBOT
			news= filterAps(getRobotAps());
			//control of user aps
			for (int i=0;i<news.size();i++)
				for (int j=0;j<robot_aps.size();j++)
					if(robot_aps.get(j).compare(news.get(i)))
					{
						robot_aps.get(j).check=true;
						news.get(i).check=true;
					}
			//update of user aps
			for (int i=0;i<robot_aps.size();i++)
				if (robot_aps.get(i).check!=true) 
				{
					final int idf=robot_aps.get(i).getId();
					myHandler.post(new Runnable(){
						@Override
						public void run() {
							myApp.removeRobotAp(idf);
						}});
				}
			for (int i=0;i<news.size();i++)
				if (news.get(i).check!=true) 
				{
					final int idf=news.get(i).getId();
					myHandler.post(new Runnable(){
						@Override
						public void run() {
								myApp.addRobotAp(idf,last_response);
						}});
				}
			robot_aps=news;
			
			//se c'è novità allora manda richiesta di refresh alla GUI - > che aggiunge le nuove in coda, mette la flag di rimozione alle altre 
			//(i.e. toglie se sono in normal, le toglie al prossimo reset se sono selezionate)
		}
		Log.e(TAG,"closing...");
	}
	
	public void halt()
	{
		Log.v(TAG,"Closing...");
		stop=true;
	}
	
	public void uncheckAll()
	{
		for (int i=0;i<user_aps.size();i++)
			user_aps.get(i).check=false;
		for (int i=0;i<robot_aps.size();i++)
			robot_aps.get(i).check=false;
		for (int i=0;i<list_aps.size();i++)
			list_aps.get(i).check=false;
	}
	
	public ArrayList<AP_couple> getFullList()
	{
		InputStream is=null;
		String result="";
		ArrayList<AP_couple> news= new ArrayList<AP_couple>();
		
		try{
			HttpParams httpParameters = new BasicHttpParams();
	    	int timeoutConnection = 5000;  //the proper connection timeout
	    	HttpConnectionParams.setConnectionTimeout(httpParameters, timeoutConnection);
	    	int timeoutSocket = 5000;      //the operation timeout
	    	HttpConnectionParams.setSoTimeout(httpParameters, timeoutSocket);
	    	
	    	HttpClient hc= new DefaultHttpClient(httpParameters); 
	    	//HttpPost hp= new HttpPost(url+"full_action_list");
	    	HttpGet hp=new HttpGet(url+"full_action_list");
	    	Log.v(TAG,"starting connection for current status...");
	    	HttpResponse response = hc.execute(hp);
	    	//Log.v(TAG,"Executed the query");
	    	HttpEntity entity = response.getEntity();
	    	is = entity.getContent();
	    	
	    	if (is!=null)
			{
				try{
					 BufferedReader reader = new BufferedReader(new InputStreamReader(is,"iso-8859-1"),8);
			         StringBuilder sb = new StringBuilder();
			         String line = null;
			         while ((line = reader.readLine()) != null) {
			        	 sb.append(line + "\n");
			         }
			         is.close();
		
			         result=sb.toString();
				}catch(Exception e)
				{
					Log.e(TAG,"error parisng string!");
				}
			}
	    	
	    	last_response=result;
	    	
	    	try{
	    		JSONArray jArray = new JSONArray(result);
	            for(int i=0;i<jArray.length();i++){
	            	JSONObject json_data = jArray.getJSONObject(i);
	                AP_couple action= new AP_couple(json_data.getInt("apId"),
	                		json_data.getDouble("likelihood"));
	                news.add(action);
	            }
	         }catch(JSONException e){
	             Log.e(TAG, "* Error parsing data "+e.toString());
	         }
	    	
		}catch(Exception e)
		{
			Log.e(TAG,"Http connection error... ");
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				Log.e(TAG,"Thread sleeping problem!");
			}
		}
		return news;
	}
	
	public ArrayList<AP_couple> getRobotAps()
	{
		InputStream is=null;
		String result="";
		ArrayList<AP_couple> news= new ArrayList<AP_couple>();
		ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();
		nameValuePairs.add(new BasicNameValuePair("uid",Integer.toString(myApp.getUid())));
		nameValuePairs.add(new BasicNameValuePair("ulang",Integer.toString(myApp.getUserLanguage())));
		
		try{
			HttpParams httpParameters = new BasicHttpParams();
	    	int timeoutConnection = 5000;  //the proper connection timeout
	    	HttpConnectionParams.setConnectionTimeout(httpParameters, timeoutConnection);
	    	int timeoutSocket = 5000;      //the operation timeout
	    	HttpConnectionParams.setSoTimeout(httpParameters, timeoutSocket);
	    	
	    	HttpClient hc= new DefaultHttpClient(httpParameters); 
	    	String myurl=url+"robot_actions";
	    	myurl+="?"+URLEncodedUtils.format(nameValuePairs,"utf-8");
	    	HttpGet hp= new HttpGet(myurl);
	    	
	    	Log.v(TAG,"starting connection for current robot aps...");
	    	HttpResponse response = hc.execute(hp);
	    	//Log.v(TAG,"Executed the query");
	    	HttpEntity entity = response.getEntity();
	    	is = entity.getContent();
	    	
	    	if (is!=null)
			{
				try{
					 BufferedReader reader = new BufferedReader(new InputStreamReader(is,"iso-8859-1"),8);
			         StringBuilder sb = new StringBuilder();
			         String line = null;
			         while ((line = reader.readLine()) != null) {
			        	 sb.append(line + "\n");
			         }
			         is.close();
		
			         result=sb.toString();
				}catch(Exception e)
				{
					Log.e(TAG,"error parisng string!");
				}
			}
	    	
	    	last_response=result;
	    	
	    	try{
	    		JSONArray jArray = new JSONArray(result);
	            for(int i=0;i<jArray.length();i++){
	            	JSONObject json_data = jArray.getJSONObject(i);
	                AP_couple action= new AP_couple(json_data.getInt("apId"),
	                		json_data.getDouble("likelihood"));
	                news.add(action);
	            }
	         }catch(JSONException e){
	             Log.e(TAG, "* Error parsing data "+e.toString());
	         }
	    	
		}catch(Exception e)
		{
			Log.e(TAG,"Http connection error... ");
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				Log.e(TAG,"Thread sleeping problem!");
			}
		}
		return news;
	}
	
	public ArrayList<AP_couple> getUserAps()
	{
		InputStream is=null;
		String result="";
		ArrayList<AP_couple> news= new ArrayList<AP_couple>();
		ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();
		nameValuePairs.add(new BasicNameValuePair("uid",Integer.toString(myApp.getUid())));
		nameValuePairs.add(new BasicNameValuePair("ulang",Integer.toString(myApp.getUserLanguage())));
		
		try{
			HttpParams httpParameters = new BasicHttpParams();
	    	int timeoutConnection = 5000;  //the proper connection timeout
	    	HttpConnectionParams.setConnectionTimeout(httpParameters, timeoutConnection);
	    	int timeoutSocket = 5000;      //the operation timeout
	    	HttpConnectionParams.setSoTimeout(httpParameters, timeoutSocket);
	    	
	    	HttpClient hc= new DefaultHttpClient(httpParameters); 
	    	//HttpPost hp= new HttpPost(url+"user_actions");
	    	String myurl=url+"user_actions";
	    	myurl+="?"+URLEncodedUtils.format(nameValuePairs,"utf-8");
	    	HttpGet hp= new HttpGet(myurl);
	    	Log.v(TAG,"starting connection for current status...");
	    	//hp.setEntity(new UrlEncodedFormEntity(nameValuePairs));
	    	HttpResponse response = hc.execute(hp);
	    	//Log.v(TAG,"Executed the query");
	    	HttpEntity entity = response.getEntity();
	    	is = entity.getContent();
	    	
	    	if (is!=null)
			{
				try{
					 BufferedReader reader = new BufferedReader(new InputStreamReader(is,"iso-8859-1"),8);
			         StringBuilder sb = new StringBuilder();
			         String line = null;
			         while ((line = reader.readLine()) != null) {
			        	 sb.append(line + "\n");
			         }
			         is.close();
		
			         result=sb.toString();
				}catch(Exception e)
				{
					Log.e(TAG,"error parisng string!");
				}
			}
	    	
	    	last_response=result;
	    	
	    	try{
	    		JSONArray jArray = new JSONArray(result);
	            for(int i=0;i<jArray.length();i++){
	            	JSONObject json_data = jArray.getJSONObject(i);
	                AP_couple action= new AP_couple(json_data.getInt("apId"),
	                		json_data.getDouble("likelihood"));
	                news.add(action);
	            }
	         }catch(JSONException e){
	             Log.e(TAG, "* Error parsing data "+e.toString());
	         }
	    	
		}catch(Exception e)
		{
			Log.e(TAG,"Http connection error... ");
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				Log.e(TAG,"Thread sleeping problem!");
			}
		}
		return news;
	}
	
	protected ArrayList<AP_couple> filterAps(ArrayList<AP_couple> aps)
	{
		while(aps.size()>myApp.getMaxActions())
    	{
    		aps.remove(aps.size()-1);
    	}
		return aps;
	}

}
