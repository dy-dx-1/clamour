#!/bin/bash
USERPASS="!clamour"

for ip in `cat ./hosts`; do
    ssh-keygen -f "/home/"$USER"/.ssh/known_hosts" -R $ip
    echo $USERPASS | sshpass ssh-copy-id -i ~/.ssh/id_rsa.pub -o StrictHostKeyChecking=no "pi@"$ip
done
