package it.unisi.accompany.widget;

import it.unisi.accompany.R;
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

public class Value extends Button implements View.OnTouchListener{

	protected final String TAG="Accompany GUI - Options";
	
	public static final int USR=1;
	public static final int RBT=2;
	
	public final int MYWIDTH=100;//90
	protected final int REDUCEDWIDTH=75;//65;
	protected final int TEXTCOLOR;
	protected final int TEXTSIZE =18;
	protected final int REDUCEDTEXTSIZE =14;
	
	protected String value;
	protected Option owner;
	
	protected AbsoluteLayout myLayout;
	
	protected int state=0; // 0 -normal 1- selected
	
	protected AnimationSet shake;
	
	protected int xx,yy,original_x,original_y;
	
	//dragging
	boolean flag_move=false;
	float myLastTouch_x,myLastTouch_y;
	int pointerId;
	protected final float PXTRESH=1f;
	private final int INVALID_POINTER_ID=-1;
	
	public Value(Context context, String val, int mode, Option ow, AbsoluteLayout layout) {
		super(context);
		this.value=val;
		this.owner=ow;
		this.myLayout=layout;
		
		if (mode==USR)
		{
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.round_robot_simple));
			TEXTCOLOR= Color.BLACK;
		}
		else
		{
			//this.setBackgroundDrawable(getResources().getDrawable(R.drawable.simple_robot_button));
			//TEXTCOLOR= context.getResources().getColor(R.color.black_transparency70);
			this.setBackgroundDrawable(getResources().getDrawable(R.drawable.round_robot_simple));
			TEXTCOLOR= Color.BLACK;
		}
		
		this.setTextColor(TEXTCOLOR);
		this.setText(value);
		this.setWidth(MYWIDTH);
		this.setHeight(MYWIDTH);
		this.setTextSize(TEXTSIZE);
		
		this.setOnTouchListener(this);
		
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
	
	public void startShakeAnimation()
	{
		this.startAnimation(shake);
	}

	public void disappear()
	{
		this.setVisibility(View.INVISIBLE);
		myLayout.removeView(this);
	}
	
	@Override
	public boolean onTouch(View v, MotionEvent event) {
		final int action=event.getAction();
		if (state==0)
		{
			switch (action & MotionEvent.ACTION_MASK)
			{
			case MotionEvent.ACTION_DOWN:
			{
				Log.d(TAG +"(Value drag)","Action down");
				flag_move=false;
				myLastTouch_x=event.getX();
				myLastTouch_y=event.getY();
				pointerId=event.getPointerId(0);
				
				myLayout.removeView(this);
				myLayout.addView(this);
				break;
			}
			case MotionEvent.ACTION_MOVE:
			{
				Log.d(TAG +"(Value drag)","Action move");
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
	
				}
				
				break;
			}
			case MotionEvent.ACTION_UP:
			{
				Log.d(TAG +"(Value drag)","Action up");
				pointerId= INVALID_POINTER_ID;
				AbsoluteLayout.LayoutParams paras = 
						(AbsoluteLayout.LayoutParams)v.getLayoutParams();
				//if (owner.isInsideMe((int)(paras.x+MYWIDTH/2),(int)( paras.y+MYWIDTH/2)))
				if (owner.isInsideMe((int)(paras.x),(int)( paras.y)))
				{
					xx=(int)(paras.x);
					yy=(int)( paras.y);
					owner.setSelectedValue(this);
					state=1;
					Log.i(TAG,"added value");
				}
				else
				{	
					paras.x=this.original_x;
					paras.y=this.original_y;
					v.setLayoutParams(paras);
				}
			break;
		}
		case MotionEvent.ACTION_CANCEL:
		{
			Log.d(TAG +"(Value drag)","Action cancel");					
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
			Log.d(TAG +"(Value drag)","Action pointer up");
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
		else
		{
			return owner.onTouch(owner,event);
		}
	}

	public String getValue()
	{
		return value;
	}
	
	public void setPosition(int xxx, int yyy)
	{
		xx=xxx;
		yy=yyy;
		original_x=xx;
		original_y=yy;
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
	
}
