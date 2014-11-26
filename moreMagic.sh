LOUISE_DIR=$(pwd)
FOLDER=$1
FILE=$2
#scp $LOUISE_DIR/$FOLDER/* student@Carelia:$FOLDER/
#ssh -t -X student@Carelia "cd $FOLDER && make && echo 'Make done' && ./Main && echo done; bash "

expect explore.exp $LOUISE_DIR $FOLDER $FILE
