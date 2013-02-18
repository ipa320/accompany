package com.questit.accompany2.widget;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.StringTokenizer;

import com.questit.accompany2.R;
import com.questit.accompany2.activities.UserView;

import android.content.Context;
import android.util.Log;
import android.view.View;
import android.widget.Button;

public class CloudSimpleButton extends Button {
	
	// default MAX & min values for width and height of the buttons.
		protected final int MAXHEIGHT=150;
		protected final int MAXWIDTH=600;
		protected final int MINHEIGHT=75;
		protected final int MINWIDTH=300;
		protected final int MINTEXTSIZE=30;
		protected final int MAXTEXTSIZE=60;
		
		//Values of the button
		protected String name;    //values for the visualization and functionality
		protected String command;
		protected double likelihood;
		protected String phrase;
		protected String original_phrase;
		protected Context cont;   //to hold the context
		protected UserView act;  //to keep contact with the main class
		protected int myHeight;  //measures of the button size (active also when it is not displayed)
		protected int myWidth;
		
		//variables for the connection
		protected Socket robot_client;
		protected String robot_ip;
		protected int robot_port;
		
		public CloudSimpleButton(Context context, String n,String cmd, double lik,String p,
				UserView a) {
			super(context);
			
			//copy of values
			cont= context;
			name=n;
			command=cmd;
			likelihood=lik;
			original_phrase=phrase=p;
			act=a;
			
			//working on the phrase, to properly encode it:

			phrase="";
			StringTokenizer st= new StringTokenizer(original_phrase," ");
			phrase=st.nextToken();
			while (st.hasMoreTokens())
			{
				phrase=phrase+"_"+st.nextToken();
			}
			
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
			
			//setting up thhe onClickListener
			this.setOnClickListener(new View.OnClickListener() {
				
				@Override
				public void onClick(View v) {
					//reset of selected parent buttons
					act.resetAllButtsFather();     
							
					if (original_phrase.contains("athccendo loothcee")) original_phrase="accendo luci";
					if (original_phrase.contains("spengo loothcee")) original_phrase="spengo luci";
						
					act.toastMessage(original_phrase);
					act.sendUserActionRequest(command, phrase);					
				}
				
				/*@Override
				public void onClick(View v) {
					//reset of selected parent buttons
					act.resetAllButtsFather();     
					//toast message with the command to be executed (debug)
					Log.e("Robot","Sending: execute "+command+" "+phrase);
					//Connecting to robot server
					try 
					{
						Log.i("INFO","socket da creare");
						robot_client=new Socket(robot_ip,robot_port);
						//COB_sock=new Socket("192.168.1.104",CareOBotPort);
						Log.i("INFO","socket creata");
						//sending the command to the robot (remote command server)
						if (robot_client!=null)
						{
							PrintWriter os = new PrintWriter(robot_client.getOutputStream());
							//os.write("fwd 1.0 5");
							os.write("execute " + command+ " "+phrase);
							//os.write(this.command);
							os.flush();
							os.close();
							robot_client.close();
							robot_client=null;
						}
						
						if (original_phrase.contains("athccendo loothcee")) original_phrase="accendo luci";
						if (original_phrase.contains("spengo loothcee")) original_phrase="spengo luci";
						
						act.showRobotExecutingCommandView(phrase);
						act.toastMessage(original_phrase);
						
					} catch (UnknownHostException e) {
						Log.e("ERR","Unknown host for the Care-o-bot");
					//toastMessage("Please set the correct Ip and port for the care-o.bot");
					} catch (IOException e) {
						Log.e("ERR","IO Exception for the Care-o-bot");
					}
					
				}*/
			});
		}

		public int getMyWidth()
		{
			return this.myWidth;
		}
		
		public int getMyHeight()
		{
			return this.myHeight;
		}
		
		public double getLikelihood()
		{
			return likelihood;
		}
		
		/*@Override
		public void setWidth()
		{
			
		}*/

}
