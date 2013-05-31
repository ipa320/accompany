package it.unisi.accompany.widget;

import java.util.ArrayList;

import it.unisi.accompany.R;
import it.unisi.accompany.clients.DatabaseClient;
import android.animation.Animator;
import android.animation.Animator.AnimatorListener;
import android.animation.AnimatorSet;
import android.animation.ObjectAnimator;
import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.animation.AnimationSet;
import android.view.animation.TranslateAnimation;
import android.widget.AbsoluteLayout;
import android.widget.Button;


public class Option extends Button implements View.OnTouchListener{
	
	protected final String TAG="Accompany GUI - Options";
	
	public static final int USR=1;
	public static final int RBT=2;
	
	public final int MYWIDTH=150;
	protected final int REDUCEDWIDTH=115;
	protected final int TEXTCOLOR;
	protected final int TEXTSIZE =24;
	protected final int REDUCEDTEXTSIZE =18;
	
	protected int id;
	protected String name="";
	protected String[] values;
	protected String selected_value;
	protected String default_value;
	
	protected ArrayList<Value> myValues;
	protected Value selectedValue;
	
	protected int state=0; //0 -not selected 1-selected 2-with a value selected
	protected int xx,yy;
	
	protected ActionPossibilityWidget owner;
	protected int mode;
	
	protected Context context;
	protected AbsoluteLayout myLayout;
	
	//dragging
	boolean flag_move=false;
	public int original_x,original_y;
	float myLastTouch_x,myLastTouch_y;
	int pointerId;
	protected final float PXTRESH=1f;
	private final int INVALID_POINTER_ID=-1;
	
	protected AnimationSet shake;

	public Option(String n, String v,String d, int i, Context cont,int mode, ActionPossibilityWidget apw, AbsoluteLayout al )
	{
		super(cont);
		id=i;
		name=n;
		values=v.split(",");
		selected_value=d;
		owner=apw;
		myLayout=al;
		context = cont;
		default_value=d;
		
		if (mode==USR)
		{
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.round_robot_simple));
			TEXTCOLOR= Color.BLACK;
		}
		else
		{
			//this.setBackgroundDrawable(getResources().getDrawable(R.drawable.simple_robot_button));
			//TEXTCOLOR= cont.getResources().getColor(R.color.black_transparency70);
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.round_robot_simple));
			TEXTCOLOR= Color.BLACK;
		}
		
		this.setTextColor(TEXTCOLOR);
		this.setText(name);
		this.setWidth(MYWIDTH);
		this.setHeight(MYWIDTH);
		this.setTextSize(TEXTSIZE);
		
		this.setOnTouchListener(this);
		
		myValues= new ArrayList<Value>();
		
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

	public int getId()
	{
		return id;
	}

	public String getName()
	{
		return name;
	}

	public String getValue(int i)
	{
		if (values.length<i)
			return values[i];
		else return null;
	}

	public String[] getValues()
	{
		return values;
	}
	
	public void startShakeAnimation()
	{
		this.startAnimation(shake);
		if (selectedValue!=null)
			selectedValue.startShakeAnimation();
	}
	
	public void setPosition(int xxx, int yyy)
	{
		xx=xxx;
		yy=yyy;
		original_x=xx;
		original_y=yy;
		//this.setX(xx);
		//this.setY(yy);
	}
	
	public void setSelected(boolean b)
	{
		if (b) state=1;
		else state=0;
	}
	
	public String getValue()
	{
		return name;
	}
	
	public int getMyId()
	{
		return id;
	}
	
	@Override
	@SuppressLint("NewApi")
	@TargetApi(11)
	public boolean onTouch(View v, MotionEvent event) {
		final int action=event.getAction();
		switch (action & MotionEvent.ACTION_MASK)
		{
		case MotionEvent.ACTION_DOWN:
		{
			Log.d("Accompany-gestureDet","Action down");
			flag_move=false;
			myLastTouch_x=event.getX();
			myLastTouch_y=event.getY();
			pointerId=event.getPointerId(0);
			
			if (state!=2)
			{
				owner.resetOptionsInShow(this);
				myLayout.removeView(this);
				myLayout.addView(this);
			}
			break;
		}
		case MotionEvent.ACTION_MOVE:
		{
			Log.d("Accompany-gestureDet","Action move");
			final int act_p=event.getPointerId(pointerId);
			final float xc= event.getX();
			final float yc= event.getY();
			final float dx= (xc-myLastTouch_x);
			float dy= (yc-myLastTouch_y);
			if (((float)Math.sqrt(dx*dx+dy*dy))>PXTRESH)
			{
				flag_move=true;
				AbsoluteLayout.LayoutParams paras = 
						(AbsoluteLayout.LayoutParams)v.getLayoutParams();
				paras.x=paras.x+(int)dx;//(int)(xc-myWidth/2);
				paras.y=paras.y+(int)dy;//(int)(yc-myWidth/2);
				v.setLayoutParams(paras);
				
				myLastTouch_x=xc-dx;//(int)x;
				myLastTouch_y=yc-dy;//(int)y;

				if (selectedValue!=null)
				{
					AbsoluteLayout.LayoutParams paras2 = 
							(AbsoluteLayout.LayoutParams)selectedValue.getLayoutParams();
					paras2.x=paras2.x+(int)dx;//(int)(xc-myWidth/2);
					paras2.y=paras2.y+(int)dy;//(int)(yc-myWidth/2);
					selectedValue.setLayoutParams(paras2);
				}
			}
			
			break;
		}
		case MotionEvent.ACTION_UP:
		{
			if ((state==1||state==2)&&!flag_move)
			{
				owner.onClick(this);
			}
			Log.d("Accompany-gestureDet","Action up");
			pointerId= INVALID_POINTER_ID;
			AbsoluteLayout.LayoutParams paras = 
					(AbsoluteLayout.LayoutParams)v.getLayoutParams();
			if (state==0)
			{
				if (owner.isInsideMe((int)(paras.x+MYWIDTH/2),(int)( paras.y+MYWIDTH/2)))
				{
					//owner.addSelection(this);
					xx=(int)(paras.x);
					yy=(int)( paras.y);
					this.valueChoice();
					Log.i(TAG,"added option");
				}
				else
				{	
					paras.x=this.original_x;
					paras.y=this.original_y;
					v.setLayoutParams(paras);
				}
			}
			else
			{
				if (!owner.isInsideMe((int)(paras.x+MYWIDTH/2),(int)( paras.y+MYWIDTH/2)))
				{
					//owner.addSelection(this);
					if (selectedValue!=null) selectedValue.setVisibility(View.INVISIBLE);
					paras.x=this.original_x;
					paras.y=this.original_y;
					xx=original_x;
					yy=original_y;
					v.setLayoutParams(paras);
					this.removeValueChoice();
					Log.i(TAG,"option removed");
				}
				else
				{	
					paras.x=this.xx;
					paras.y=this.yy;
					v.setLayoutParams(paras);
				}
			}
			break;
		}
		case MotionEvent.ACTION_CANCEL:
		{
			Log.d("Accompany-gestureDet","Action cancel");					
			pointerId= INVALID_POINTER_ID;			
			AbsoluteLayout.LayoutParams paras = 
					(AbsoluteLayout.LayoutParams)v.getLayoutParams();
			paras.x=this.original_x;
			paras.y=this.original_y;
			v.setLayoutParams(paras);
			break;
		}
		case MotionEvent.ACTION_POINTER_UP:
		{
			Log.d("Accompany-gestureDet","Action pointer up");
			final int p_idx=(event.getAction() & MotionEvent.ACTION_POINTER_INDEX_MASK) >> 
			MotionEvent.ACTION_POINTER_INDEX_SHIFT;
			final int ppppp= event.getPointerId(p_idx);
			if (ppppp==pointerId)
			{
				final int newPointer = p_idx == 0 ? 1 : 0;
				myLastTouch_x=(int) event.getX();
				myLastTouch_y=(int) event.getY();
				pointerId=event.getPointerId(newPointer);
			}		
			break;
		}
		}
		return true;
	}
	
	protected void showMyValues()
	{
		int x,y;
		double butt_radius,next_r;
		int margine=5;
		int offset=65;
		double raggio=
				owner.getCurrentWidth()/2+offset;//MYHLWIDTH+5; 
		double angolo=3*Math.PI/2;
		x= owner.getHLX()+(int)(raggio*Math.cos(angolo));
		y= owner.getHLY()+(int)(raggio*Math.sin(angolo));
		boolean flag;
		int idx=0;
		butt_radius=0;
		for(int j=0;j<myValues.size();j++)
		{
			Value actual= myValues.get(j);
			int x1,y1;
			x1=x-((int)(actual.MYWIDTH/2));
			y1=y-((int)(actual.MYWIDTH/2));
			actual.setPosition(x1, y1);
			AbsoluteLayout.LayoutParams pa= new AbsoluteLayout.LayoutParams(AbsoluteLayout.LayoutParams.WRAP_CONTENT, AbsoluteLayout.LayoutParams.WRAP_CONTENT, x1,y1);
			actual.setLayoutParams(pa);
			myLayout.addView(actual);
				
			butt_radius=actual.MYWIDTH/2;
			if (j==myValues.size()-1)
				next_r=0;
			else
			{
				next_r=myValues.get(j+1).MYWIDTH/2;
				//angolo=angolo+(2*Math.asin((butt_radius + margine+ next_r)/(2*raggio)));
				//x= owner.getHLX()+(int)(raggio*Math.cos(angolo));
				//y= owner.getHLY()+(int)(raggio*Math.sin(angolo));
			}
			angolo=angolo+(2*Math.asin((butt_radius + margine+ next_r)/(2*raggio)));
			x= owner.getHLX()+(int)(raggio*Math.cos(angolo));
			y= owner.getHLY()+(int)(raggio*Math.sin(angolo));
		}
	}
	
	protected void valueChoice()
	{
		owner.grow(true);
		startReduce();
		state=1;	
		
		//create the values array
		for (int j=0;j<values.length;j++)
			myValues.add(new Value(context, values[j], mode, this,myLayout));
		
		showMyValues();
		
		owner.checkState();
	}
	
	protected void removeValueChoice()
	{
		owner.grow(false);
		clearReduction();
		
		if (state==2)
			myLayout.removeView(selectedValue);
		else if (state==1)
			 	for (int i=0;i<myValues.size();i++)
			 		myLayout.removeView(myValues.get(i));
			
		state=0;
			
		//destroy the value array
		myValues.clear();
		selected_value=default_value;
		selectedValue=null;
		
		owner.checkState();
	}
	
	public boolean isSelected()
	{
		Log.i(TAG,"isselected "+name+"?"+" -> state:"+state);
		if (state!=2) return false;
		else return true;
	}
	
	public boolean isInsideMe(int x, int y)
	{
		int distance=(int)Math.sqrt((x-xx)*(x-xx)+(y-yy)*(y-yy));
		if (distance<(REDUCEDWIDTH/2))
			return true;
		else 
			return false;
	}
	
	public void setSelectedValue(Value v)
	{
		selectedValue=v;
		for (int i=0;i<myValues.size();i++)
	 		if (!myValues.get(i).equals(v)) myLayout.removeView(myValues.get(i));
		myValues.clear();
		state=2;
		selected_value=v.getValue();
		v.startReduce();
		Log.i(TAG,"setted selected. state: "+state);
		owner.checkState();
	}
	
	public int getMyWidth()
	{
		if (state==0) return MYWIDTH;
		else return REDUCEDWIDTH;
	}
	
	@TargetApi(11)
	@SuppressLint("NewApi")
	public void startReduce()
	{
		AnimatorSet reduction= new AnimatorSet();
		ObjectAnimator growX=ObjectAnimator.ofFloat(this, "x", xx,xx+Math.abs(MYWIDTH-REDUCEDWIDTH)/2);
		ObjectAnimator growY=ObjectAnimator.ofFloat(this, "y", yy,yy+Math.abs(MYWIDTH-REDUCEDWIDTH)/2);
		ObjectAnimator growW=ObjectAnimator.ofInt(this, "width", MYWIDTH,REDUCEDWIDTH);
		ObjectAnimator growH=ObjectAnimator.ofInt(this, "height", MYWIDTH,REDUCEDWIDTH);
		ObjectAnimator textS=ObjectAnimator.ofFloat(this, "textSize", TEXTSIZE,REDUCEDTEXTSIZE);
		growH.setDuration(200);growY.setDuration(200);growX.setDuration(200);growW.setDuration(200);textS.setDuration(200);
		reduction.playTogether(growH,growW,growX,growY,textS);
		reduction.start();
	}
	
	@TargetApi(11)
	@SuppressLint("NewApi")
	public void clearReduction()
	{
		AnimatorSet reduction= new AnimatorSet();
		ObjectAnimator growX=ObjectAnimator.ofFloat(this, "x", xx+Math.abs(MYWIDTH-REDUCEDWIDTH)/2,xx);
		ObjectAnimator growY=ObjectAnimator.ofFloat(this, "y", yy+Math.abs(MYWIDTH-REDUCEDWIDTH)/2,yy);
		ObjectAnimator growW=ObjectAnimator.ofInt(this, "width", REDUCEDWIDTH,MYWIDTH);
		ObjectAnimator growH=ObjectAnimator.ofInt(this, "height",REDUCEDWIDTH,MYWIDTH);
		ObjectAnimator textS=ObjectAnimator.ofFloat(this, "textSize",REDUCEDTEXTSIZE,TEXTSIZE);
		growH.setDuration(200);growY.setDuration(200);growX.setDuration(200);growW.setDuration(200);textS.setDuration(200);
		reduction.playTogether(growH,growW,growX,growY,textS);
		reduction.start();
	}
	
	public void disappear()
	{
		if (selectedValue!=null) selectedValue.disappear();
		AnimatorSet reduction= new AnimatorSet();
		ObjectAnimator growX=ObjectAnimator.ofFloat(this, "x", xx+Math.abs(MYWIDTH-REDUCEDWIDTH)/2, xx+REDUCEDWIDTH/2);
		ObjectAnimator growY=ObjectAnimator.ofFloat(this, "y", yy+Math.abs(MYWIDTH-REDUCEDWIDTH)/2, yy+REDUCEDWIDTH/2);
		ObjectAnimator growW=ObjectAnimator.ofInt(this, "width", REDUCEDWIDTH,0);
		ObjectAnimator growH=ObjectAnimator.ofInt(this, "height",REDUCEDWIDTH,0);
		growH.setDuration(200);growY.setDuration(200);growX.setDuration(200);growW.setDuration(200);
		reduction.playTogether(growH,growW,growX,growY);
		
		reduction.addListener(new AnimatorListener() {
			@Override
			public void onAnimationStart(Animator animation) {
			}
			@Override
			public void onAnimationRepeat(Animator animation) {
			}
			@Override
			public void onAnimationEnd(Animator animation) {
				myLayout.removeView(Option.this);
			}
			@Override
			public void onAnimationCancel(Animator animation) {
			}
		});
		
		reduction.start();
	}
	
	public void resetMe()
	{	
		this.setVisibility(View.GONE);
		if (state==2)
		{
			this.clearAnimation();
			selectedValue.clearAnimation();
			selectedValue.setVisibility(View.INVISIBLE);
			myLayout.removeView(selectedValue);
		}
		else if (state==1)
			 	for (int i=0;i<myValues.size();i++)
			 		myLayout.removeView(myValues.get(i));
		
		myValues.clear();
		selected_value=default_value;
		selectedValue=null;
		state=0;
		
		Log.e("rrr","reset me, opion : "+ name+",state:"+state);
		myLayout.removeView(this);
		
	}
	
	public void resetMeIfInShow()
	{
		if (state==1)
		{
			for (int i=0;i<myValues.size();i++)
		 		myLayout.removeView(myValues.get(i));
			state=0;
			selected_value=default_value;
			selectedValue=null;
			myValues.clear();
			
			final float old_x=this.getX();
			final float old_Y=this.getY();
			
			AnimatorSet reduction= new AnimatorSet();
			ObjectAnimator growX=ObjectAnimator.ofFloat(this, "x", xx+Math.abs(MYWIDTH-REDUCEDWIDTH)/2,original_x);
			ObjectAnimator growY=ObjectAnimator.ofFloat(this, "y", yy+Math.abs(MYWIDTH-REDUCEDWIDTH)/2,original_y);
			ObjectAnimator growW=ObjectAnimator.ofInt(this, "width", REDUCEDWIDTH,MYWIDTH);
			ObjectAnimator growH=ObjectAnimator.ofInt(this, "height",REDUCEDWIDTH,MYWIDTH);
			growH.setDuration(200);growY.setDuration(200);growX.setDuration(200);growW.setDuration(200);
			reduction.playTogether(growH,growW,growX,growY);
			
			reduction.addListener(new AnimatorListener() {
				@Override
				public void onAnimationStart(Animator animation) {
				}
				@Override
				public void onAnimationRepeat(Animator animation) {
				}
				@Override
				public void onAnimationEnd(Animator animation) {
					Option.this.setX(old_x);Option.this.setY(old_Y);
					AbsoluteLayout.LayoutParams paras = 
							(AbsoluteLayout.LayoutParams)Option.this.getLayoutParams();
					paras.x=Option.this.original_x;
					paras.y=Option.this.original_y;
					Option.this.setLayoutParams(paras);
				}
				@Override
				public void onAnimationCancel(Animator animation) {
				}
			});
			
			reduction.start();
			owner.grow(false);
			
		}
	}
	
	public void setParameter(DatabaseClient db)
	{
		db.requestParameterSet(id,selected_value);
	}
	
	public void removeFromLayout()
	{
		Log.e("rrr","removeing from layout: "+name);
		if (state==2)
		{
			this.clearAnimation();
			selectedValue.clearAnimation();
			selectedValue.setVisibility(View.INVISIBLE);
			myLayout.removeView(selectedValue);
		}
		else if (state==1)
		{
		 	for (int i=0;i<myValues.size();i++)
		 	{
		 		myValues.get(i).setVisibility(View.GONE);
		 		myLayout.removeView(myValues.get(i));
		 	}
		}
		this.setVisibility(View.INVISIBLE);
		myLayout.removeView(this);
	}

}
