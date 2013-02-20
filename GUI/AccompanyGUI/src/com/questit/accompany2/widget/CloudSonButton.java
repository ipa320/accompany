package com.questit.accompany2.widget;


import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.StringTokenizer;

import com.questit.accompany2.R;
import com.questit.accompany2.activities.UserView;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.View;
import android.widget.AbsoluteLayout;
import android.widget.Button;

public class CloudSonButton extends Button implements View.OnClickListener{
	// default MAX & min values for width and height of the buttons.
		protected final int MAXHEIGHT=100;
		protected final int MAXWIDTH=400;
		protected final int MINHEIGHT=50;
		protected final int MINWIDTH=200;
		protected final int MINTEXTSIZE=20;
		protected final int MAXTEXTSIZE=40;
		
		protected final int MYTEXTCOLOR=Color.parseColor("#111111");
		
		protected String name;
		protected int x;
		protected int y;
		protected CloudFatherButton parent;
		
		protected UserView act;
		
		protected String command;
		protected String phrase;
		protected String original_phrase;
		
		protected String CareOBotIP;
		protected int CareOBotPort;
		
		protected Socket COB_sock;
		
		protected int myHeight;
		protected int myWidth;
		
		
		public CloudSonButton(Context context,String Name, String command,String phr, int plus_x,int plus_y, CloudFatherButton pparent,
				UserView a)
		{
			super(context);
			this.name=Name;
			this.setText(name);
			this.setTextSize(9);
			//this.setTypeface(null,Typeface.BOLD);
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.cloud_son_button_shape));
			this.setPadding(5+((int)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*pparent.getLikelihood())), 0,
					5+((int)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*pparent.getLikelihood())), 0);
			//Computation of button width and height based on the likelihood:
			myHeight=(MINHEIGHT+(int)((MAXHEIGHT-MINHEIGHT)*pparent.getLikelihood()));
			myWidth=(MINWIDTH+(int)((MAXWIDTH-MINWIDTH)*pparent.getLikelihood()));
			this.setWidth(myWidth);
			this.setHeight(myHeight);
			//and of text size:
			this.setTextSize((float)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*pparent.getLikelihood()));
			//this.setTextColor(Color.parseColor("#006699"));
			
			this.setOnClickListener(this);
			this.COB_sock=null;
			this.parent=pparent;
			this.x=parent.getPosX()+plus_x;
			this.y=parent.getPosY()+plus_y;
			this.command=command;
			this.original_phrase=this.phrase=phr;
			//working on the phrase, to properly encode it:
			phrase="";
			StringTokenizer st= new StringTokenizer(original_phrase," ");
			phrase=st.nextToken();
			while (st.hasMoreTokens())
			{
				phrase=phrase+"_"+st.nextToken();
			}
			
			this.act=a;
			
			//this.setTextColor(Color.BLUE);
			this.setTextColor(MYTEXTCOLOR);
		}


		@Override
		public void onClick(View v) {
			if (original_phrase.equals("athccendo lootchee")) original_phrase="accendo luci";
			if (original_phrase.equals("spengo lootchee")) original_phrase="spengo luci";
							
			//calling the coreect procedure to set the robot busy
			act.toastMessage(original_phrase);	
			act.sendUserActionRequest(command, phrase);
							
			//call the method to return the layout in the original state--> reset the parent!
			parent.resetSons();
		}
		
		/*@Override
		public void onClick(View v) {
			//Connecting to robot server
			Log.e("Robot","Sending: execute "+command+" "+phrase);
					try 
					{
						Log.i("INFO","socket da creare ");
						COB_sock=new Socket(CareOBotIP,CareOBotPort);
						//COB_sock=new Socket("192.168.1.104",CareOBotPort);
						Log.i("INFO","socket creata");
						//sending the command to the robot (remote command server)
						if (COB_sock!=null)
						{
							Log.e("connnnn","clicked");
							Log.i("conn","connected" +
									"");
							PrintWriter os = new PrintWriter(COB_sock.getOutputStream());
							os.write("execute " + command+" "+phrase);
							//os.write(this.command);
							os.flush();
							os.close();
							COB_sock.close();
							COB_sock=null;
							
							if (original_phrase.equals("athccendo lootchee")) original_phrase="accendo luci";
							if (original_phrase.equals("spengo lootchee")) original_phrase="spengo luci";
							
							//calling the coreect procedure to set the robot busy
							act.showRobotExecutingCommandView(phrase);
							//act.robotBusy();
							act.toastMessage(original_phrase);
						}
					} catch (UnknownHostException e) {
						Log.e("ERR","Unknown host for the Care-o-bot");
					//toastMessage("Please set the correct Ip and port for the care-o.bot");
					} catch (IOException e) {
						Log.e("ERR","IO Exception for the Care-o-bot");
					}
					
					//call the method to return the layout in the original state--> reset the parent!
					parent.resetSons();
					//parent.remove();  //cava il parent dopo invio comando (poi altre opzioni appariranno
					//a condition aggiornata)
		}*/
		
		public int getMyX()
		{
			return this.x;
		}
		
		public int getMyY()
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
		
		public int getMyHeight()
		{
			return myHeight;
		}
		
		public int getMyWidth()
		{
			return myWidth;
		}

}
