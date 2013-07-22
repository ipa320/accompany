package it.unisi.accompany.widget.actionliststuffs;

import android.content.Context;
import android.graphics.Color;
import android.view.Gravity;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.FrameLayout.LayoutParams;

public class HorizontalEnvLine extends FrameLayout{

	public HorizontalEnvLine(Context context, int color,int length) {
		super(context);
		Button b= new Button(context);
		b.setClickable(false);
		b.setBackgroundColor(color);
		b.setHeight(2);
		b.setWidth(length);
		this.setBackgroundColor(Color.TRANSPARENT);
		FrameLayout.LayoutParams p= new FrameLayout.LayoutParams(LayoutParams.MATCH_PARENT,1);
		p.gravity=Gravity.CENTER;
		b.setLayoutParams(p);
		this.addView(b);
	}

}
