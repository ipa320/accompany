package it.unisi.accompany.widget.actionliststuffs;


import it.unisi.accompany.activities.ActionsListView;
import android.content.Context;
import android.graphics.Color;
import android.view.View;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.LinearLayout.LayoutParams;

public class ActionsEnvironmentButton extends LinearLayout
{
	protected final int HIGHLIGHTED = Color.parseColor("#33AABB"); 
	protected final int TEXTSIZE=24;
	
	protected Context cont;
	protected String environment;
	protected ActionsListView act;
	//protected String robotIp;
	//protected int robot_port;
	//protected Socket COB_sock;
	protected ActionsEnvironmentButton me;
	
	protected View.OnClickListener myOnClickListener;
	protected TextView tv;

	public ActionsEnvironmentButton(Context context, String env,ActionsListView ac) {
		super(context);
		
		this.setBackgroundColor(Color.TRANSPARENT);
		this.setClickable(true);
		tv=new TextView(context);
		tv.setTextColor(Color.WHITE);
		tv.setTextSize(TEXTSIZE);
		tv.setPadding(15,10,0,5);
		tv.setBackgroundColor(Color.TRANSPARENT);
		this.setBackgroundColor(Color.TRANSPARENT);
		//this.setBackgroundDrawable(getResources().getDrawable(R.drawable.my_gradient_button));
		//this.setBackgroundColor(Color.MAGENTA);
		this.environment=env;
		this.act=ac;
		tv.setText(environment);
		tv.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT));
		this.addView(tv);
		
		me=this;
		//this.robotIp=ip;
		//this.robot_port=port;
		//this.COB_sock=null;
		
		this.myOnClickListener= new View.OnClickListener() {
			
			@Override
			public void onClick(View v) {
				//act.toastMessage(environment);
				act.showActionsFor(environment);
				me.setBackgroundColor(HIGHLIGHTED);
			}
		};
		
		this.setOnClickListener(myOnClickListener);
		
	}
	
	public String getEnvironment()
	{
		return this.environment;
	}
	
	public void setHighlighted()
	{
		this.setBackgroundColor(HIGHLIGHTED);
	}
}
