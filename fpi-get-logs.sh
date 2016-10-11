#!/bin/bash

# FTP settings
HOST="217.36.211.159"
USER="fpi_$1"
PASS="$2"

# Backup the existing log data
rm -r logs-old
mv logs logs-old
rm latest-logs.tar.gz~
mv latest-logs.tar.gz latest-logs.tar.gz~

# Download the latest logs from the FTP
ftp -n -v $HOST <<END_FTP
user $USER $PASS
binary
get latest-logs.tar.gz
quit
END_FTP

# Decompress the logs
mkdir logs
cd logs
tar -xzvf ../latest-logs.tar.gz

