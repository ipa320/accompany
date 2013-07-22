package it.unisi.accompany.widget.actionliststuffs;

import it.unisi.accompany.clients.DatabaseClient;

import java.util.ArrayList;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

public class OptionLayout extends LinearLayout {

	protected Context cont;
	
	protected String name;
	protected String[] values;
	protected String default_value;
	protected String selected_value;
	protected int myid;
	protected ArrayList<Button> ValuesButtons;
	protected ActionButton owner;
	
	public OptionLayout(Context context,String n,String v,String def_v,int idf,ActionButton ab) {
		super(context);
		this.name=n;
		this.cont=context;
		this.values=v.split(",");
		this.default_value=def_v;
		this.myid=idf;
		this.owner=ab;
		this.selected_value=default_value;
		this.ValuesButtons= new ArrayList<Button>();
		
		this.setOrientation(LinearLayout.HORIZONTAL);
		this.setBackgroundColor(Color.TRANSPARENT);
		
		this.setUpView();
		
		LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
		p.setMargins(10, 2, 10, 2);
		this.setLayoutParams(p);
	}
	
	protected void setUpView()
	{
		//set up option name:
		TextView tv= new TextView(cont);
		tv.setTextColor(Color.WHITE);
		tv.setBackgroundColor(Color.TRANSPARENT);
		tv.setTextSize(20);
		tv.setText(name+":");
		LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
		p.setMargins(10, 5, 10, 5);
		tv.setLayoutParams(p);
		this.addView(tv);
		
		//set up options values (highligthing the default one)
		for (int i=0;i<values.length;i++)
		{
			Button b= new Button(cont);
			b.setTextColor(Color.WHITE);
			if (values[i].equals(default_value))
				b.setBackgroundColor(Color.parseColor("#FF005969"));
			else
				b.setBackgroundColor(Color.TRANSPARENT);
			//b.setTextColor();
			b.setText(values[i]);
			b.setOnClickListener(new View.OnClickListener() {
				@Override
				public void onClick(View v) {
					OptionLayout.this.resetButtons();
					v.setBackgroundColor(Color.parseColor("#FF005969"));
					Button bb= (Button)v;
					selected_value=bb.getText().toString();
					Log.e("MARCO",selected_value);
				}
			});
			p = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
			tv.setLayoutParams(p);
			p.setMargins(5, 5, 5, 5);
			b.setLayoutParams(p);
			
			this.addView(b);
			
			ValuesButtons.add(b);
		}
		
	}
	
	public void resetButtons()
	{
		for (int i=0;i<ValuesButtons.size();i++)
			ValuesButtons.get(i).setBackgroundColor(Color.TRANSPARENT);
	}
	
	public void setParameter(DatabaseClient db)
	{
		db.requestParameterSet(myid,selected_value);
	}

}
