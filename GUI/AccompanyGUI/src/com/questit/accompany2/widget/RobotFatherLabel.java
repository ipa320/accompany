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
import com.questit.accompany2.activities.RobotView;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.View;
import android.widget.AbsoluteLayout;
import android.widget.Button;

public class RobotFatherLabel extends Button implements View.OnClickListener
{
	protected final int MYCOLORHIGHLIGTHED = Color.parseColor("#006699");
	protected final int MYCOLOR = Color.BLACK;
	//protected final int MYTEXTSIZE = 24;
	
	// default MAX & min values for width and height of the buttons.
	protected final int MAXHEIGHT=75;
	protected final int MAXWIDTH=300;
	protected final int MINHEIGHT=35;
	protected final int MINWIDTH=150;
	protected final int MINTEXTSIZE=20;
	protected final int MAXTEXTSIZE=30;
	
	protected int ap_id;
	protected double likelihood;
	
	protected ArrayList<RobotSonLabel> sons;
	
	protected String name;
	protected boolean state;
	
	protected int x;
	protected int y;
	protected int myHeight;
	protected int myWidth;
	
	protected Socket COB_sock;
	
	protected Context cont;
	
	protected RobotView act;
	@SuppressWarnings("deprecation")
	protected AbsoluteLayout myLayout;
	
	public RobotFatherLabel(Context context,int apid,String Name,String command,double lik,int pos_x,int pos_y,
			@SuppressWarnings("deprecation") AbsoluteLayout layout,
			RobotView a) {
		super(context);
		this.setPadding(15, 5, 15, 5);
		this.likelihood=lik;
		
		//IL COMMAND NON SI USA PERCH� DEVE SOLO FARE SPAWN DEI SONS
		this.ap_id=apid;
		this.name=Name;
		this.setText(name);
		this.setTextColor(MYCOLOR);
		
		//this.setTypeface(null,Typeface.BOLD);
		//this.setTextSize(MYTEXTSIZE);
		//this.setBackgroundColor(Color.TRANSPARENT);
		//this.setBackgroundDrawable(getResources().getDrawable(R.drawable.label_bck));
		this.setBackgroundDrawable(getResources().getDrawable(R.drawable.my_shape));
		this.setOnClickListener(this);
		state=true;
		x=100;
		y=50;
		COB_sock=null;
		
		
		this.cont=context;
		this.myLayout=layout;
		
		sons= new ArrayList<RobotSonLabel>();
		this.x=pos_x;
		this.y=pos_y;
		
		this.act=a;
		
		myHeight=(MINHEIGHT+(int)((MAXHEIGHT-MINHEIGHT)*likelihood));
		myWidth=(MINWIDTH+(int)((MAXWIDTH-MINWIDTH)*likelihood));
		this.setWidth(myWidth);
		this.setHeight(myHeight);
		this.setTextSize((float)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*likelihood));
	}

	/*@Override
	public void onClick(View v)
	{
		//change the state:
		this.setTextColor(MYCOLORHIGHLIGTHED);
		this.state=false;
		//this.setTextColor(HIGHLIGHTED);
		//query to get the sons
		InputStream is=null;
        String result="";
        //data to send
        ArrayList<NameValuePair> nameValuePairs = new ArrayList<NameValuePair>();
        nameValuePairs.add(new BasicNameValuePair("id",Integer.toString(ap_id)));
        
        Log.e("aaa",Integer.toString(ap_id));
        
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
        	HttpPost hp= new HttpPost("http://"+Database_ip+"/getSons.php");
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
                RobotSonLabel son= new RobotSonLabel(cont,json_data.getString("ap_label"),
                		json_data.getString("command"),json_data.getString("phraseal_feedback"),
                		createX(i),createY(i),this,robot_ip,act,robot_port);
                Log.e("aaa",json_data.getString("ap_label"));
                //add the son to its own sons list:
                sons.add(son);
        		//add the son to the list of views active in activity:
        		this.act.addSonLabel(son);
            }
        }catch(JSONException e){
            Log.e("log_tag", "Error parsing data "+e.toString());
        }
        // Spawn the sons:
        for (int i=0;i<sons.size();i++)
        {
        	sons.get(i).autoSetParams();
        	myLayout.addView(sons.get(i));
        }
		this.setClickable(false);
		//act.refreshMask();
	}*/
	
	@Override 
	public void onClick(View v)
	{
		this.setTextColor(MYCOLORHIGHLIGTHED);
		this.state=false;
		act.sendSonRequest(ap_id,this);
	}
	
	public void setSons(String res)
	{
		Log.i("app-resnse","rfl");
		 //parse json data
        try{
            JSONArray jArray = new JSONArray(res);
            for(int i=0;i<jArray.length();i++){
                JSONObject json_data = jArray.getJSONObject(i);
                RobotSonLabel son= new RobotSonLabel(cont,json_data.getString("ap_label"),
                		json_data.getString("command"),json_data.getString("phraseal_feedback"),
                		createX(i),createY(i),this,act);
                Log.e("aaa",json_data.getString("ap_label"));
                //add the son to its own sons list:
                sons.add(son);
        		//add the son to the list of views active in activity:
        		this.act.addSonLabel(son);
            }
        }catch(JSONException e){
            Log.e("log_tag", "Error parsing data "+e.toString());
        }
        // Spawn the sons:
        for (int i=0;i<sons.size();i++)
        {
        	sons.get(i).autoSetParams();
        	myLayout.addView(sons.get(i));
        }
		this.setClickable(false);
		//act.refreshMask();
	}
	
	//� una sorta di callback chiamata sulla chiusura dell'action del son clickato
	public void resetSons()
	{
		for (int i=0;i<sons.size();i++)
		{
			myLayout.removeView(sons.get(i));
			this.act.removeSonLabel(sons.get(i));
		}
		sons.clear();
		this.state=true;
		this.setTextColor(MYCOLOR);
		this.setClickable(true);
	}
	
	public int getPosX()
	{
		return this.x;
	}
	
	public int getPosY()
	{
		return this.y;
	}
	
	//method that generates static position of sons around the parent
	public int createX(int pos) {
		if (pos==0)
			return 50;
		else
			if (pos==1)
				return -125;
			else
				if (pos==2)
					return 125;
				else return 50;
	}
	
	public int createY(int pos)
	{
		if (pos==0)
			return 25;
		else
			if (pos==1)
				return -25;
			else
				if (pos==2)
					return -25;
				else return 35*(pos-1);
	}
	
	public void randomPose(int sizeX, int sizeY)
	{
		sizeX=Math.max(sizeX-200,0);
		sizeY=Math.max(sizeY-100,0);
		double r1=Math.random();
		double r2=Math.random();
		x=50+(int)(r1*sizeX);
		y=50+(int)(r2*sizeY);
	}
	
	public void setPosition(int xx, int yy)
	{
		this.x=xx;
		this.y=yy;
	}
	
	public AbsoluteLayout.LayoutParams getParams()
	{		
		AbsoluteLayout.LayoutParams p=new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x,y);
		return p;
	}
	
	public void autoSetParams(int w)
	{
		AbsoluteLayout.LayoutParams p=getParams();
		p.width=w;
		this.setLayoutParams(p);
	}
	
	public void autoSetParams()
	{
		AbsoluteLayout.LayoutParams p=getParams();
		this.setLayoutParams(p);
	}

}
