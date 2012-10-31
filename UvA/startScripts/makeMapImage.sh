montage ./map.png -geometry +200+200 bigmap.png
convert -crop 200x200+515+320 ./bigmap.png cut.png
convert ./cut.png -resize 380x380 final.png
