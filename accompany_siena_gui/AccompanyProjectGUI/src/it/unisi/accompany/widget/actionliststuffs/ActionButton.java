package it.unisi.accompany.widget.actionliststuffs;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import it.unisi.accompany.R;
import it.unisi.accompany.activities.ActionsListView;
import it.unisi.accompany.widget.ActionPossibilityWidget;
import it.unisi.accompany.widget.Option;
import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.FrameLayout.LayoutParams;

public class ActionButton extends FrameLayout {
	
	protected final String TAG = "AccompanyGUIActionListButton";
	protected final int TEXTSIZE=20;
	
	protected final int MAX_OPTIONS=4;
	
	protected final int NORMAL=0;
	protected final int SHOWOPTIONS=1;
	
	protected int state;
	
	protected TextView tv;
	protected ActionsListView act;
	protected int ap_id;
	protected String name;
	protected String phrase,original_phrase;
	protected int act_precondition_id;
	protected Context cont;
	
	protected ArrayList<OptionLayout> options;
	protected View.OnClickListener myListener;
	
	
	public ActionButton(Context context,String n,String p,
			ActionsListView ac,int id, int precond_id) 
	{
		super(context);
		this.cont=context;
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
				act.sendOptionsRequest(ap_id, ActionButton.this);
			}
		};
		
		this.setOnClickListener(myListener);
	}
	
	public void sendCommand()
	{
		act.toastMessage(original_phrase);
		act.sendActionListActionRequest(act_precondition_id);//(command, phrase);
	}
	
	public void handleOptionsResponse(String r, int id)
	{
		if (id==ap_id)
		{
			this.tv.setTextColor(Color.parseColor("#FF005969"));
			options= new ArrayList<OptionLayout>();
			 try{
		            JSONArray jArray = new JSONArray(r);
		            for(int i=0;(i<jArray.length()&&i<MAX_OPTIONS);i++){
		                JSONObject json_data = jArray.getJSONObject(i);
		                OptionLayout opt= new OptionLayout(cont,json_data.getString("name"),
		                		json_data.getString("values"),json_data.getString("default"),
		                		Integer.parseInt(json_data.getString("id")),this);
		               options.add(opt);
		            }
		        }catch(JSONException e){
		            Log.e("log_tag", "Error parsing data "+e.toString());
		        }
			 
			 showOptions();
			 state=SHOWOPTIONS;
			 
		}
		else
			Log.i(TAG,"options response when button state was resetted... not showing options");
	}
	
	public void showOptions()
	{
	    LinearLayout popupView= new LinearLayout(cont);
	    popupView.setOrientation(LinearLayout.VERTICAL);
	    popupView.setBackgroundColor(Color.BLACK);
	    popupView.setPadding(2, 3, 2, 3);
	    
	    TextView title= new TextView(cont);
	    title.setText("Options: ");
	    title.setTextColor(Color.WHITE);
	    title.setTextSize(22);
	    LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
	    title.setLayoutParams(p);
	    p.setMargins(10, 5, 0, 15);
	    popupView.addView(title);
	    
	    for (int i=0;i<options.size();i++)
	    {	    	
	    	popupView.addView(options.get(i));
	    }
	    
	    FrameLayout fl = new FrameLayout(cont);
	    Button ok= new Button(cont);
	    ok.setBackgroundColor(Color.parseColor("#FF323232"));
	    ok.setText(R.string.ok);
	    FrameLayout.LayoutParams pp = new FrameLayout.LayoutParams(FrameLayout.LayoutParams.WRAP_CONTENT,
	    		FrameLayout.LayoutParams.WRAP_CONTENT, Gravity.CENTER_HORIZONTAL);
	    ok.setLayoutParams(pp);
	    ok.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				sendOptionsSelected();
				ActionButton.this.sendCommand();
			}
		});
	    fl.addView(ok);
	    	    
	    p = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
	    p.setMargins(0, 10, 0, 10);
	    fl.setLayoutParams(p);
	    popupView.addView(fl);

	    act.showOptionsPd(popupView, this);
	}
	
	public void reset()
	{
		this.tv.setTextColor(Color.WHITE);
	}
	
	protected void sendOptionsSelected()
	{
		for (int i=0;i<options.size();i++)
			options.get(i).setParameter(act.getDbClient());
	}

}
