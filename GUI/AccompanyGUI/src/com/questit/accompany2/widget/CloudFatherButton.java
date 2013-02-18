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
import com.questit.accompany2.activities.UserView;

import android.content.Context;
import android.graphics.Color;
import android.graphics.Rect;
import android.util.Log;
import android.view.View;
import android.widget.AbsoluteLayout;
import android.widget.Button;

public class CloudFatherButton extends Button implements View.OnClickListener{
	
	// default MAX & min values for width and height of the buttons.
		protected final int MAXHEIGHT=150;
		protected final int MAXWIDTH=600;	
		protected final int MINHEIGHT=75;
		protected final int MINWIDTH=300;
		protected final int MINTEXTSIZE=30;
		protected final int MAXTEXTSIZE=60;
		
		protected double SPAWN_AREA_FACTOR_WIDTH=1.25;
		protected double SPAWN_AREA_FACTOR_HEIGHT=1.75;
		
		protected int MAX_NUMBER_OF_SONS=4;
		
		protected final int MYTEXTCOLOR= Color.BLACK;
		protected final int HIGHLIGHTEDTEXT = Color.parseColor("#006699");
		
		
		
		protected int ap_id;
		
		protected ArrayList<CloudSonButton> sons;
		protected double likelihood;
		protected String name;
		protected boolean state;
		
		protected int x;
		protected int y;
		protected int myHeight;
		protected int myWidth;
		
		protected Socket COB_sock;
		protected String robot_ip;
		protected int robot_port;
		protected String database_ip;
		
		protected Context cont;
		
		protected UserView act;
		@SuppressWarnings("deprecation")
		protected AbsoluteLayout myLayout;
		
		protected int son_visualization_offset;
		protected int y_flag=0;
		protected int y_pos=0;
		
		public CloudFatherButton(Context context,int apid,String Name,double lik,
				@SuppressWarnings("deprecation") AbsoluteLayout layout,
				UserView a) {
			super(context);
			
			//copy of values
			this.cont=context;
			this.myLayout=layout;	
			sons= new ArrayList<CloudSonButton>();
			this.act=a;
			this.ap_id=apid;
			this.name=Name;
			this.likelihood=lik;
			
			//setting up the layout of the button:
			this.setText(name);
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.my_shape));
			this.setPadding(5+((int)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*likelihood)), 0,
					5+((int)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*likelihood)), 0);
			
			//Check if the likelihood is a valid value:
			if (likelihood<0.) 
			{
				likelihood=0.;
				act.toastMessage("WARNING: Negative likelihood button "+name);
			}
			if (likelihood>1.)
			{
				likelihood=1.;
				act.toastMessage("WARNING: Likelihood greater than 1 for button "+name);
			}
				
			//Computation of button width and height based on the likelihood:
			myHeight=(MINHEIGHT+(int)((MAXHEIGHT-MINHEIGHT)*likelihood));
			myWidth=(MINWIDTH+(int)((MAXWIDTH-MINWIDTH)*likelihood));
			this.setWidth(myWidth);
			this.setHeight(myHeight);
			//and of text size:
			this.setTextSize((float)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*likelihood));
			//this.setTextColor(Color.parseColor("#006699"));
			this.setTextColor(MYTEXTCOLOR);
			this.setOnClickListener(this);
			state=true; //state (clickable, not selected)
			COB_sock=null;
			
			this.son_visualization_offset=0;
		}
		
		/*@Override
		public void onClick(View v)
		{
			//change the state:
					//this.setTextColor(Color.RED);
					act.resetAllButtsFather();
					this.setTextColor(HIGHLIGHTEDTEXT);
			
					this.state=false;
					//act.toastMessage(this.name);
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
			        	HttpPost hp= new HttpPost("http://"+database_ip+"/getSons.php");
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
			                CloudSonButton son= new CloudSonButton(cont,json_data.getString("ap_label"),
			                		json_data.getString("command"),json_data.getString("phraseal_feedback"),0,
			                		0,this,robot_ip,act,robot_port);
			                Log.e("aaa",json_data.getString("ap_label"));
			                //add the son to its own sons list:
			                sons.add(son);
			        		//add the son to the list of views active in activity:
			        		//this.act.addSonLabel(son);
			            }
			        }catch(JSONException e){
			            Log.e("log_tag", "Error parsing data "+e.toString());
			        }
			        filterSons();
			        addSonsToLayout();
					this.setClickable(false);
		}*/
		@Override
		public void onClick(View v)
		{
			act.sendSonRequest(ap_id, this);
		}
		
		public void setSons(String res)
		{
			act.resetAllButtsFather();
			this.setTextColor(HIGHLIGHTEDTEXT);
			this.state=false;
			
			 //parse json data
	        try{
	            JSONArray jArray = new JSONArray(res);
	            for(int i=0;i<jArray.length();i++){
	                JSONObject json_data = jArray.getJSONObject(i);
	                CloudSonButton son= new CloudSonButton(cont,json_data.getString("ap_label"),
	                		json_data.getString("command"),json_data.getString("phraseal_feedback"),0,
	                		0,this,act);
	                Log.e("aaa",json_data.getString("ap_label"));
	                //add the son to its own sons list:
	                sons.add(son);
	        		//add the son to the list of views active in activity:
	        		//this.act.addSonLabel(son);
	            }
	        }catch(JSONException e){
	            Log.e("log_tag", "Error parsing data "+e.toString());
	        }
	        filterSons();
	        addSonsToLayout();
			this.setClickable(false);
		}
		
		//remove sons exceding the parameter threshold:
		protected void filterSons()
		{
			//now we assume sons ordered by likelihood and thus we remove the last ones
			while (sons.size()>MAX_NUMBER_OF_SONS)
				sons.remove(sons.size()-1);	
		}
		
		//spawns sons in the layout avoiding collisions
		protected void addSonsToLayout()
		{
			//setting up the spawn area around the father button
			int spawn_area_width=(int)(SPAWN_AREA_FACTOR_WIDTH*this.getMyWidth());
			int spawn_area_height=(int)(SPAWN_AREA_FACTOR_HEIGHT*this.getHeight());
			int minx=this.x-(int)((spawn_area_width/2));
			int miny=this.y-(int)(((SPAWN_AREA_FACTOR_HEIGHT-1)*this.getMyHeight()/2));
			//assuring the viibility of that area!
			if ((minx+spawn_area_width+this.getMyWidth())>(act.getWindowManager().getDefaultDisplay().getWidth()-act.getBannerWidth()))
				minx=minx-((minx+spawn_area_width+this.getMyWidth())-(act.getWindowManager().getDefaultDisplay().getWidth()-act.getBannerWidth()));
			if ((miny+spawn_area_height+this.getMyHeight())>(act.getWindowManager().getDefaultDisplay().getHeight()))
				miny=miny-((miny+spawn_area_height+this.getMyHeight())-act.getWindowManager().getDefaultDisplay().getHeight());
			if (minx<act.getBannerWidth()) minx=act.getBannerWidth();
			if (miny<0) miny=0;
			//Cycle attemping to spawn all the sons
			boolean done=false;
	    	
	    	while(!done)
	    	{
	    		int attemps=0;
	    		done=true;
	    	
	    		for (int i=0;i<sons.size();i++)
	    		{
	    			boolean flag=true;
	    			while(flag==true)
	    			{
	    				flag=false;
	    				//setting up the coordinates of the sons:
					
	    				int xx = minx+(int)(Math.random()*spawn_area_width);  //xhdpi
	    				int yy = miny+(int)(Math.random()*spawn_area_height);
	    				sons.get(i).setPosition(xx, yy);
	    				sons.get(i).autoSetParams();
	    				myLayout.addView(sons.get(i));
	    				Rect r1;
	    				int x1=((AbsoluteLayout.LayoutParams)sons.get(i).getLayoutParams()).x;
	    				int y1=((AbsoluteLayout.LayoutParams)sons.get(i).getLayoutParams()).y;
	    				r1= new Rect(x1,y1,(x1+sons.get(i).getMyWidth()),(y1+sons.get(i).getMyHeight()));
	    				//seeking possible collisions
	    				for (int h=0;h<i;h++)
	    				{
	    					Rect r2;
	    					int x2=((AbsoluteLayout.LayoutParams)sons.get(h).getLayoutParams()).x;
	    					int y2=((AbsoluteLayout.LayoutParams)sons.get(h).getLayoutParams()).y;
	    					r2= new Rect(x2,y2,(x2+sons.get(h).getMyWidth()),(y2+sons.get(h).getMyHeight()));
	    					//r2= new Rect(x2,y2,(x2+Butt.get(h).getLayoutParams().getMyWidth()),(y2+Butt.get(h).getMyHeight()));
	    					//act.toastMessage(""+Butt.get(h).getLayoutParams().width);
	    					//Log.i("Info","h:"+h +" rect 2:"+r2.left+", "+r2.top+", "+r2.right+", "+r2.bottom);
	    					if (Rect.intersects(r1, r2)) 
	    					{
	    						flag= true;
	    						Log.i("Info","rectangles "+i+" and "+h+" intersecate!");
	    					}
	    				}
	    				if (flag) 
	    				{
	    					myLayout.removeView(sons.get(i));
	    					if (attemps>25)
	    					{
	    						for (int h=0;h<i;h++)
	    							myLayout.removeView(sons.get(h));
	    						done=false;
	    						i=sons.size();
	    						
	        					flag=false;
	    					}
	    					attemps++;
	    					//act.toastMessage("evviva");
	    				}
	    			}
	        	}
	    	}
		}
		
		public void resetSons()
		{
			if (state==false)
			{
				for (int i=0;i<sons.size();i++)
				{
					myLayout.removeView(sons.get(i));
					//this.act.removeSonCloudButtons(sons.get(i));
				}
				sons.clear();
				this.state=true;
				//this.setTextColor(Color.parseColor("#006699"));
				this.setClickable(true);
				this.setTextColor(MYTEXTCOLOR);
			}
		}
		
		public int getPosX()
		{
			return this.x;
		}
		
		public int getPosY()
		{
			return this.y;
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
			
		public int getMyWidth()
		{
			return myWidth;
		}
			
		public int getMyHeight()
		{
			return myHeight;
		}
			
		public double getLikelihood()
		{
			return likelihood;
		}

}
