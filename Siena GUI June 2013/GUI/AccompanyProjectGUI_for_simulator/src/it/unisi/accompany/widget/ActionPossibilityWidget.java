package it.unisi.accompany.widget;

import it.unisi.accompany.activities.AccompanyActivity;
import it.unisi.accompany.R;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


import android.animation.Animator;
import android.animation.AnimatorSet;
import android.animation.ObjectAnimator;
import android.animation.Animator.AnimatorListener;
import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Color;
import android.graphics.drawable.AnimationDrawable;
import android.media.MediaPlayer;
import android.util.Log;
import android.view.View;
import android.view.animation.AnimationSet;
import android.view.animation.TranslateAnimation;
import android.widget.AbsoluteLayout;
import android.widget.Button;
import android.widget.ImageView;

public class ActionPossibilityWidget extends Button implements View.OnClickListener {
	
	protected final String TAG = "apw";//"Accompany GUI - Action Possibility";
	
	//CONSTANTS
	protected final int MAXHEIGHT=220;              // default MAX & min values for width and height of the buttons.
	protected final int MAXWIDTH=220;//300;
	protected final int MINHEIGHT=140;
	protected final int MINWIDTH=140;
	protected final int MINTEXTSIZE=26;//24//30
	protected final int MAXTEXTSIZE=34;//60
	
	protected final int MYHLWIDTH=225;//200;              // width when hilighted
	protected final int HLX=850;                    // highlighted position
	protected final int HLY=376;
		
	protected final int GROWSTEP=20;                // growing step when eat an option
	
	protected final int running_X=1100;             // running values
	protected final int running_Y=650;
	protected final int MYRUNNINGWIDTH=100;
	protected final int MYRUNNINGTEXTSIZE=14;
	
	protected int MAX_OPTIONS=4;                    // max supported options
	protected int MAX_VALUES =4;                    // max supported values for each option
	
	
	protected boolean is_going=false;
	
	//CONSTANTS FOR VIEWS
	protected final int USER  = 1;
	protected final int ROBOT = 2;
	
	//flag to see if it's in robot view or in user view
	int VIEW=0;   //1 - user // 2 -robot
	
	//possible state values
	protected final int NORMAL =1;
	protected final int SHOWOPTIONS =2;
	protected final int WAITCONF =3;
	protected final int RUNNING = 4;
	
	//Values of the Action Possibility
	protected int ap_id;                            // action possibility id;
	protected String name;    //values for the visualization and functionality
	protected double likelihood;
	protected String phrase;
	protected String original_phrase;
	protected int act_precondition_id;             //this is the "command"
	
	protected ArrayList<Option> options;
	
	protected int state;
	
	//to manage the removal of an old AP only if it's actually unused in the last few seconds!
	protected boolean to_be_removed;
	
	//button values
	protected int x;
	protected int y;
	protected int current_x;
	protected int current_y;
	protected int my_X,my_Y;
	protected int myWidth;
	protected int current_width;
	protected float myTextSize;
	
	protected Context cont;     //the context
	protected AccompanyActivity act;
	
	protected AbsoluteLayout myLayout;  // the layout where the button is displayed
	protected MediaPlayer mp;
	
	protected int son_visualization_offset;
	protected int y_flag=0;
	protected int y_pos=0;
	
	protected AnimatorSet myAnimator;
	protected ImageView my_blue_circle;
	
	protected AnimationSet shake;
	protected AnimatorSet grow;
	protected AnimationDrawable colourAnim;
	protected AnimatorSet goRunning;
	
	//dragging stuffs
	protected View.OnTouchListener OptionsDragListener;
	boolean flag_move=false;
	protected int original_x,original_y;
	float myLastTouch_x,myLastTouch_y;
	int pointerId;
	protected final float PXTRESH=10f;
	private final int INVALID_POINTER_ID=-1;
	protected View dragging;
	
	public ActionPossibilityWidget(Context context,int apid,String Name,double lik,String frase,
			@SuppressWarnings("deprecation") AbsoluteLayout layout,
			AccompanyActivity a,int precondid,boolean UorR) {
		super(context);
		
		if (UorR)
			VIEW=USER;
		else
			VIEW=ROBOT;
		
		//copy of values
		this.cont=context;
		this.myLayout=layout;	
		this.act=a;
		this.ap_id=apid;
		this.name=Name;
		this.likelihood=lik;
		this.original_phrase=this.phrase=frase;
		this.act_precondition_id=precondid;
		this.mp= MediaPlayer.create(act,R.raw.action_possibility_click);

		//setting up the layout of the button:
		this.setText(name);
		if (VIEW==USER)
		{
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.simple_user_button));
			this.setTextColor(Color.BLACK);
		}
		else
		{
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.simple_robot_button));
			this.setTextColor(Color.BLACK);
		}
		
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
		myWidth=(MINWIDTH+(int)((MAXWIDTH-MINWIDTH)*likelihood));
		this.setWidth(myWidth);
		this.setHeight(myWidth);//myHeight);
		current_width=myWidth;
		//and of text size:
		myTextSize = (float)(MINTEXTSIZE+(MAXTEXTSIZE-MINTEXTSIZE)*likelihood);
		this.setTextSize(myTextSize);

		this.setOnClickListener(this);
		this.state=NORMAL; //state (clickable, not selected)
		
		
		this.son_visualization_offset=0;
		
		my_blue_circle = new ImageView(context);
		my_blue_circle.setBackgroundColor(Color.TRANSPARENT);
		my_blue_circle.setImageDrawable(context.getResources().getDrawable(R.drawable.blue_circle));
		my_blue_circle.setVisibility(View.INVISIBLE);
		my_blue_circle.setMinimumWidth(myWidth+40);//setWidth(myWidth+40);
		my_blue_circle.setMinimumHeight(myWidth+40);//setHeight(myWidth+40);
		my_blue_circle.setMaxHeight(myWidth+40);
		my_blue_circle.setMaxWidth(myWidth+40);
		
		//create shake animation
		shake= new AnimationSet(true);
		TranslateAnimation t1= new TranslateAnimation(0, +5,0,0);
		TranslateAnimation t2= new TranslateAnimation(+5,-5,0,0);
		TranslateAnimation t3= new TranslateAnimation(-5,+5,0,0);
		TranslateAnimation t4= new TranslateAnimation(+5,-5,0,0);
		TranslateAnimation t5= new TranslateAnimation(-5,0,0,0);
		
		t1.setDuration(50);t2.setDuration(100);t3.setDuration(100);
		t4.setDuration(100);t5.setDuration(100);
		t1.setStartOffset(1000);t2.setStartOffset(1050);t3.setStartOffset(1150);
		t4.setStartOffset(1250);t5.setStartOffset(1350);
		shake.addAnimation(t1);shake.addAnimation(t2);shake.addAnimation(t3);
		shake.addAnimation(t4);shake.addAnimation(t5);
	}
	
	public void addToLayout(int pos_x, int pos_y)
	{
		AbsoluteLayout.LayoutParams par;
		par= new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, pos_x, pos_y);
		this.setLayoutParams(par);
		my_X=pos_x;
		my_Y=pos_y;
		current_x=my_X;
		current_y=my_Y;
		myLayout.addView(this);
		par = new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, pos_x-20, pos_y-20);
		my_blue_circle.setLayoutParams(par);
		myLayout.addView(my_blue_circle);
	}
	
	@Override
	@SuppressLint("NewApi")
	@TargetApi(11)
	public void onClick(View v)
	{
		Log.i("apw","apw ("+name+", state:"+state+") on click - is clickable? "+this.isClickable());
		if (state==NORMAL)
		{
			//this.setBackgroundResource(R.drawable.round_robot_simple);
			this.setClickable(false);
			act.isMovingButton(true);
			
			mp.start();
			
			Log.e("1","1");
			myLayout.removeView(ActionPossibilityWidget.this);
			myLayout.removeView(ActionPossibilityWidget.this.my_blue_circle);
			ActionPossibilityWidget.this.addToLayout(my_X,my_Y);				
			
			act.resetAllActionsPossibilities();
			act.setClicked(this);

			if (VIEW==USER) this.setBackgroundResource(R.drawable.round_animated_button_to_white);
			else this.setBackgroundResource(R.drawable.button_transparent_to_white);
			colourAnim=(AnimationDrawable) this.getBackground();
			
			this.state=SHOWOPTIONS;
			
			my_blue_circle.setVisibility(View.VISIBLE);
			my_blue_circle.setAlpha(1f);
			current_x=HLX-(int)(MYHLWIDTH/2);//(myWidth/2);
			current_y=HLY-(int)(MYHLWIDTH/2);//(myWidth/2);
			ObjectAnimator animMoveX = ObjectAnimator.ofFloat(ActionPossibilityWidget.this,"x",my_X,current_x );
			ObjectAnimator animMoveY = ObjectAnimator.ofFloat(ActionPossibilityWidget.this,"y",my_Y,current_y );
			ObjectAnimator animSizeW = ObjectAnimator.ofInt(ActionPossibilityWidget.this, "width", myWidth,MYHLWIDTH);
			ObjectAnimator animSizeH = ObjectAnimator.ofInt(ActionPossibilityWidget.this, "height", myWidth,MYHLWIDTH);
			animMoveX.setDuration(1000);
			animMoveY.setDuration(1000);
			animSizeW.setDuration(1000);
			animSizeH.setDuration(1000);
			AnimatorSet animMove= new AnimatorSet();
			animMove.playTogether(animMoveX,animMoveY,animSizeH,animSizeW);
			ObjectAnimator animCircle = ObjectAnimator.ofFloat(ActionPossibilityWidget.this.my_blue_circle, "alpha", 1f, 0f );
			
			animCircle.setStartDelay(200);
			animCircle.setDuration(80);
			current_width=MYHLWIDTH;
			
			myAnimator = new AnimatorSet();
			myAnimator.playSequentially(animCircle,animMove);
			
			myAnimator.addListener(new AnimatorListener(){

				@Override
				public void onAnimationCancel(Animator animation) {
					Log.i("apw","Apw ("+name+","+state+") cancelling animation!");
				}

				@Override
				public void onAnimationEnd(Animator animation) {
					Log.i("apw","apw - on animation end ("+name+"), state: "+state+" - is clickable? "+ ActionPossibilityWidget.this.isClickable());
					is_going=false;
					if (state==SHOWOPTIONS)
					{	
						ActionPossibilityWidget.this.setClickable(true);
						//colourAnim.start();
						act.sendOptionsRequest(ap_id, ActionPossibilityWidget.this);
						act.isMovingButton(false);
					}
					else
					{
						//resetMe();
						startGoBack();
					}
				}

				@Override
				public void onAnimationRepeat(Animator animation) {
					// TODO Auto-generated method stub
					
				}

				@Override
				public void onAnimationStart(Animator animation) {
					// TODO Auto-generated method stub
					
				}});
			
			is_going=true;
			myAnimator.start();
			colourAnim.start();
			
		}
		else
		{
			if (state==WAITCONF)
			{
				Log.i("TAG","Sending command: "+name);
				this.clearAnimation();
				act.resetAllActionsPossibilities(this);
				
				//executing the command
				act.toastMessage(original_phrase);
				sendOptionsSelected();                        //setting The options in the db
				act.sendActionRequest(act_precondition_id);
				
				this.state=RUNNING;
				my_blue_circle.setVisibility(View.INVISIBLE);
				
				//cleaning the options
				for (int i=0;i<options.size();i++)
					options.get(i).disappear();
				
				myLayout.invalidate();
				
				//create go to running animator
				goRunning= new AnimatorSet();
				ObjectAnimator runAnimX = ObjectAnimator.ofFloat(ActionPossibilityWidget.this,"x",current_x,running_X);
				ObjectAnimator runAnimY = ObjectAnimator.ofFloat(ActionPossibilityWidget.this,"y",current_y, running_Y);
				ObjectAnimator runAnimW = ObjectAnimator.ofInt(ActionPossibilityWidget.this, "width",MYHLWIDTH,MYRUNNINGWIDTH);
				ObjectAnimator runAnimH = ObjectAnimator.ofInt(ActionPossibilityWidget.this, "height",MYHLWIDTH,MYRUNNINGWIDTH);
				ObjectAnimator runAnimText = ObjectAnimator.ofFloat(ActionPossibilityWidget.this, "textSize", myTextSize,MYRUNNINGTEXTSIZE);
				runAnimX.setDuration(500);
				runAnimY.setDuration(500);
				runAnimH.setDuration(500);
				runAnimW.setDuration(500);
				runAnimText.setDuration(500);
				goRunning.playTogether(runAnimX,runAnimY,runAnimH,runAnimW,runAnimText);
				
				goRunning.start();
				current_x=running_X;
				current_y=running_Y;
				act.commandRunning(true);
				//act.waitRunning();
			}
			else //showing options
			{
				this.resetMe();
				
			}
		}
	}
	
	protected void sendOptionsSelected()
	{
		for (int i=0;i<options.size();i++)
			options.get(i).setParameter(act.getDbClient());
	}
	
	public void setMyPos(int xxx, int yyy)
	{
		this.my_X=xxx;
		this.my_Y=yyy;
	}
	
	public int getPosX()
	{
		return this.x;
	}
	
	public int getPosY()
	{
		return this.y;
	}
	
	//remove options exceding the parameter threshold:
	protected void filterOptions()
	{
		
	}
	
	//reset the button
	public void resetMe()
	{
		Log.i(TAG,"resetting Action Possibility ("+this.name+") state: "+ state);
		//Log.i("apw","resetting Action Possibility ("+this.name+") state: "+state);
		my_blue_circle.setVisibility(View.INVISIBLE);
		
		if (state!=NORMAL)
		{
			if (options!=null)
				for (int i=0;i<options.size();i++)
					options.get(i).resetMe();
			else Log.e(TAG,"options null");
			
			if (VIEW==USER)this.setBackgroundResource(R.drawable.round_animated_button_to_grey);
			else this.setBackgroundResource(R.drawable.button_transparent_to_grey);
			colourAnim=(AnimationDrawable) this.getBackground();
			
			if (this.myAnimator!=null) 
				if (this.myAnimator.isRunning()) 
				{
					this.myAnimator.cancel();
				}
			this.state=NORMAL;
			//this.setTextColor(Color.parseColor("#006699"));
			this.setClickable(true);
			
			if (!is_going) startGoBack();
		}
		myLayout.invalidate();
	}
	
	public void startGoBack()
	{
		myAnimator= new AnimatorSet();
		
		ObjectAnimator animMoveX = ObjectAnimator.ofFloat(ActionPossibilityWidget.this,"x",current_x,my_X);
		ObjectAnimator animMoveY = ObjectAnimator.ofFloat(ActionPossibilityWidget.this,"y",current_y, my_Y );
		ObjectAnimator animSizeW = ObjectAnimator.ofInt(ActionPossibilityWidget.this, "width",current_width,myWidth);
		ObjectAnimator animSizeH = ObjectAnimator.ofInt(ActionPossibilityWidget.this, "height",current_width,myWidth);
		current_width=myWidth;
		current_x=my_X;
		current_y=my_Y;
		animSizeW.setDuration(700);
		animSizeH.setDuration(700);
		animMoveY.setDuration(700);
		animMoveX.setDuration(700);
		animSizeW.setStartDelay(100);animSizeH.setStartDelay(100);
		animMoveY.setStartDelay(100);animMoveX.setStartDelay(100);
		myAnimator.playTogether(animMoveX,animMoveY,animSizeW,animSizeH);
		myAnimator.start();
		colourAnim.start();
	}
	
	//reset the options 
	protected void resetOptions()
	{
		
	}
	
	@Override
	public void onAnimationEnd()
	{
		if (state==WAITCONF)
		{
			this.startAnimation(shake);
			for (int i=0;i<options.size();i++ )
				options.get(i).startShakeAnimation();
		}
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
		return myWidth;
	}
		
	public double getLikelihood()
	{
		return likelihood;
	}
	
	public void removeFromLayout()
	{
		if (options!=null)
			for (int i=0;i<options.size();i++)
			{
				options.get(i).removeFromLayout();
			}
		this.setVisibility(View.INVISIBLE);
		myLayout.removeView(this);
		myLayout.removeView(this.my_blue_circle);
		/*for (int i=0;i<shadows.size();i++)
			myLayout.removeView(shadows.get(i));
		shadows.clear();*/
	}
	
	public boolean isInsideMe(int x, int y)
	{
		int distance=(int)Math.sqrt((x-HLX)*(x-HLX)+(y-HLY)*(y-HLY));
		if (distance<(current_width/2))
			return true;
		else 
			return false;
	}
	
	public void resetOptionsInShow(Option o)
	{
		for (int i=0;i<options.size();i++)
			if (!options.get(i).equals(o)) options.get(i).resetMeIfInShow();
	}
	
	public void handleOptionsResponse(String res, int id)
	{
		if (state==SHOWOPTIONS&&id==ap_id)
		{
			if (options!=null)
			{
				for (int i=0;i<options.size();i++)
					options.get(i).resetMe();
			}
			myLayout.invalidate();
			
			options= new ArrayList<Option>();
			 try{
		            JSONArray jArray = new JSONArray(res);
		            for(int i=0;(i<jArray.length()&&i<MAX_OPTIONS);i++){
		                JSONObject json_data = jArray.getJSONObject(i);
		                Option opt= new Option(json_data.getString("name"),
		                		json_data.getString("values"),json_data.getString("default"),
		                		Integer.parseInt(json_data.getString("id")),cont,VIEW,this,myLayout);
		               options.add(opt);
		            }
		        }catch(JSONException e){
		            Log.e("log_tag", "Error parsing data "+e.toString());
		        }
			 
			 showOptions();
			 
			 if (options.isEmpty())
			 {
				 state=WAITCONF;
				 this.clearAnimation();
				 this.startAnimation(shake);
			 }
		}
		else
			Log.i(TAG,"Options response when in status - "+state+" - probabily resetted, not show options!");
	}
	
	public void checkState()
	{
		int state2=WAITCONF;
		for (int i=0;i<options.size();i++ )
			if (!options.get(i).isSelected()) state2=SHOWOPTIONS;
		state=state2;
		if (state==WAITCONF)
		{
			this.clearAnimation();
			this.startAnimation(shake);
			for (int i=0;i<options.size();i++ )
				options.get(i).startShakeAnimation();
		}
		else
			this.clearAnimation();
	}
	
	//animation method to be called when an option is dragged inside or oudside the view
	public void grow(boolean verso)
	{
		ObjectAnimator growX,growY,growW,growH;
		if (verso)
		{
			growX=ObjectAnimator.ofFloat(this, "x", current_x,current_x-GROWSTEP/2);
			growY=ObjectAnimator.ofFloat(this, "y", current_y,current_y-GROWSTEP/2);
			growW=ObjectAnimator.ofInt(this, "width", current_width,current_width+GROWSTEP);
			growH=ObjectAnimator.ofInt(this, "height", current_width,current_width+GROWSTEP);
		}
		else
		{
			growX=ObjectAnimator.ofFloat(this, "x", current_x,current_x+GROWSTEP/2);
			growY=ObjectAnimator.ofFloat(this, "y", current_y,current_y+GROWSTEP/2);
			growW=ObjectAnimator.ofInt(this, "width", current_width,current_width-GROWSTEP);
			growH=ObjectAnimator.ofInt(this, "height", current_width,current_width-GROWSTEP);
		}
		growH.setDuration(200);growY.setDuration(200);growX.setDuration(200);growW.setDuration(200);
		grow= new AnimatorSet();
		grow.playTogether(growH,growW,growX,growY);
		grow.addListener(new AnimatorListener(){

			@Override
			public void onAnimationCancel(Animator animation) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onAnimationEnd(Animator animation) {
				if (state==WAITCONF)
				{
					ActionPossibilityWidget.this.startAnimation(shake);
					for (int i=0;i<options.size();i++ )
						options.get(i).startShakeAnimation();
				}
			}

			@Override
			public void onAnimationRepeat(Animator animation) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onAnimationStart(Animator animation) {
				// TODO Auto-generated method stub
				
			}});
		
		grow.start();
		if (verso)
		{
			current_x-=(int)(GROWSTEP/2);
			current_y-=(int)(GROWSTEP/2);
			current_width+=GROWSTEP;
		}
		else
		{
			current_x+=(int)(GROWSTEP/2);
			current_y+=(int)(GROWSTEP/2);
			current_width-=GROWSTEP;
		}
	}
	
	public int getCurrentWidth()
	{
		return current_width;
	}
	
	public int getHLY()
	{
		return HLY;
	}
	
	public int getHLX()
	{
		return HLX;
	}
	
	public void showOptions()
	{
		int x,y;
		double butt_radius,next_r;
		int margine=5;
		double raggio=
				current_width;//MYHLWIDTH+5; //considero anche i son dello stesso size, senno è MYHLWIDTH/2+MAXSONWIDTH/2 + margine
		double angolo=Math.PI/2;
		x= HLX+(int)(raggio*Math.cos(angolo));
		y= HLY+(int)(raggio*Math.sin(angolo));
		boolean flag;
		int i=0;
		if (options.size()<1) flag=false;
		else flag=true;
		while(flag)
		{
			Option  actual;
			//special case: only one son
			actual=options.get(i);
				
			int x1,y1;
			x1=x-((int)(actual.getMyWidth()/2));
			y1=y-((int)(actual.getMyWidth()/2));
			actual.setPosition(x1, y1);
			AbsoluteLayout.LayoutParams pa= new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x1,y1);
			actual.setLayoutParams(pa);
			myLayout.addView(actual);
			
			butt_radius=actual.getMyWidth()/2;
			if (i==options.size()-1)
				next_r=0;
			else
				next_r=options.get(i+1).getMyWidth()/2;
			angolo=angolo+(2*Math.asin((butt_radius + margine+ next_r)/(2*raggio)));
			x= HLX+(int)(raggio*Math.cos(angolo));
			y= HLY+(int)(raggio*Math.sin(angolo));
			
			i++;
			if(i==options.size()) flag=false;
		}
	}
	  public String getName()
	    {
	    	return name;
	    }
	
	  public void setRunningTask()
	  {
		  if (state!=NORMAL&&state!=RUNNING)
		  {
			  this.clearAnimation();
			  this.resetMe();
		  }
		  this.setClickable(false);
	  }
	  
}
