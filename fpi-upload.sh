#!/bin/bash

# FTP settings
HOST="217.36.211.159"
USER="fpi_$1"
PASS="$2"

# Backup the previous version
rm formulapi.tar.gz~
mv formulapi.tar.gz formulapi.tar.gz~

# Compress the folder ignoring some files
tar -czvf formulapi.tar.gz -X tar.exclude *

# Upload the new version to the FTP
ftp -n -v $HOST <<END_FTP
user $USER $PASS
binary
delete formulapi.tar.gz~
rename formulapi.tar.gz formulapi.tar.gz~
put formulapi.tar.gz
quit
END_FTP


