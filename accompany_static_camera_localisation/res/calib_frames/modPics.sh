# test

c=0
files=`ls *.jpg`
for i in $files
do
  c=$( expr $c + 1 )
  mod=$(expr $c % 10)
  echo $i $mod
  if [ $mod -eq 0 ]
  then
    echo "keep"
  else
    command=`echo "mv $i "$i"_OLD"`
    echo $command
    $command
  fi
done
