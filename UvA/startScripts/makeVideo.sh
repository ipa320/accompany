cam1=`ls | grep cam1 | sort`
cam3=`ls | grep cam3 | sort`
track=`ls | grep track | sort`

# read time stamps in arrays
unset cam1Time
for i in $cam1
do
    element=`echo ${i:0:21}`
    cam1Time=("${cam1Time[@]}" "$element")
done

unset cam3Time
for i in $cam3
do
    element=`echo ${i:0:21}`
    cam3Time=("${cam3Time[@]}" "$element")
done

unset trackTime
for i in $track
do
    element=`echo ${i:0:21}`
    trackTime=("${trackTime[@]}" "$element")
done

# get highest first timestamp of all sets
time=${cam1Time[0]}
if [ ${cam3Time[0]} -gt $time ]
then
    time=${cam3Time[0]}
fi
if [ ${trackTime[0]} -gt $time ]
then
    time=${trackTime[0]}
fi

# montage images based on timestemps
i_cam1=0
i_cam3=0
i_track=0
i=0;
while [ 1 ]
do

    while [ $time -gt ${cam1Time[$i_cam1]} ]
    do
        i_cam1=`expr $i_cam1 + 1`
    done

    while [ $time -gt ${cam3Time[$i_cam3]} ]
    do
        i_cam3=`expr $i_cam3 + 1`
    done

    while [ $time -gt ${trackTime[$i_track]} ]
    do
        i_track=`expr $i_track + 1`
    done

    echo "$i_cam1 $i_cam3 $i_track"
    time=`expr $time + 500000000`


    if [ $i_cam1 -eq ${#cam1Time[@]} ] 
    then
        break
    fi
    if [ $i_cam3 -eq ${#cam3Time[@]} ] 
    then
        break
    fi
    if [ $i_track -eq ${#trackTime[@]} ] 
    then
        break
    fi
    
    nr=`printf %07d $i`
    comp=`echo "composite"$nr".ppm"`
    cam1Img=`echo ${cam1Time[$i_cam1]}"cam1.png"`
    cam3Img=`echo ${cam3Time[$i_cam3]}"cam3.png"`
    trackImg=`echo ${trackTime[$i_track]}"track.png"`
    
    command=`echo "montage -tile 2x2 $cam1Img $cam3Img $trackImg -geometry 380x380+1+1 $comp"`
    echo $command
    `$command`
    

    i=`expr $i + 1`
done

# make video with montage images
avconv -qscale 1 -r 20 -b 960000 -i composite%07d.png humanTracking.mp4

