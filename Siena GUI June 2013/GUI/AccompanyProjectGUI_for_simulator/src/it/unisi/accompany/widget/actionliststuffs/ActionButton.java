package it.unisi.accompany.widget.actionliststuffs;

import java.util.StringTokenizer;

import it.unisi.accompany.activities.ActionsListView;
import android.content.Context;
import android.graphics.Color;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.TextView;
import android.widget.FrameLayout.LayoutParams;

public class ActionButton extends FrameLayout {
	
protected final int TEXTSIZE=20;	
	
	protected TextView tv;
	protected ActionsListView act;
	protected int ap_id;
	protected String name;
	protected String phrase,original_phrase;
	protected int act_precondition_id;
	
	protected View.OnClickListener myListener;
	
	public ActionButton(Context context,String n,String p,
			ActionsListView ac,int id, int precond_id) 
	{
		super(context);
		this.name=n;
		this.phrase=this.original_phrase=p;
		this.ap_id=id;
		this.act_precondition_id=precond_id;
		this.setBackgroundColor(Color.TRANSPARENT);
		this.setClickable(true);
		tv= new TextView(context);
		tv.setTextColor(Color.WHITE);
		tv.setTextSize(TEXTSIZE);
		tv.setPadding(2,4,2,4);
		tv.setBackgroundColor(Color.TRANSPARENT);
		tv.setText(name);
		
		tv.setLayoutParams(new LayoutParams(LayoutParams.WRAP_CONTENT,LayoutParams.WRAP_CONTENT));
		this.addView(tv);
		
		this.act=ac;
		
		//working on the phrase, to properly encode it:
		StringTokenizer st= new StringTokenizer(phrase," ");
		phrase=st.nextToken();
		while (st.hasMoreTokens())
		{
			phrase=phrase+"_"+st.nextToken();
		}
		
		myListener= new View.OnClickListener() {
			
			@Override
			public void onClick(View v) {
				
				act.toastMessage(original_phrase);
				act.sendActionListActionRequest(act_precondition_id);//(command, phrase);
			}
		};
		
		this.setOnClickListener(myListener);
	}

}
