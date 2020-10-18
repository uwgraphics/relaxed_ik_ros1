DEST="/home/haochen/Projects/RIK/collision_IK_test_output"

rm -rf $DEST/$1
rm -f $DEST/$1.zip
mkdir $DEST/$1
mkdir $DEST/$1/rmoo
mkdir $DEST/$1/rmob
cp -a ./rmoo_files/$2/* $DEST/$1/rmoo
cp -a ./rmob_files/$2/* $DEST/$1/rmob

code $DEST