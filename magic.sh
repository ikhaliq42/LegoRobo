LOUISE_DIR=$(pwd)
scp $LOUISE_DIR/Vision/* student@Carelia:Vision/
ssh -t -X student@Carelia "cd Vision && make blink && echo made && ./blinky && echo done; bash "
ssh -t -X student@Carelia "echo 'Yay!'"

