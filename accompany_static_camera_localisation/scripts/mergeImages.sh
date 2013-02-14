#
#
# mergeImages <src-dir> <dest-dir>
#

srcDir=$1
destDir=$2

echo "dest: $destDir"
echo "src: $srcDir"

max=0
files=`ls $destDir`
for i in $files
do
    base=`basename $i`
    base="${base%.*}"
    if [ $base -gt $max ]
    then 
        max=$base
    fi
done

echo "max number: $max"
max=`expr $max + 1`

files=`ls $srcDir`
for i in $files
do
    newName=`printf %04d.jpg $max`
    echo "moving $srcDir/$i to $destDir/$newName"
    mv $srcDir/$i $destDir/$newName
    max=`expr $max + 1`
done
