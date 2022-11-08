#!/bin/bash
read -p "Enter your nextcloud username: " name
read -s -p "Enter your nextcloud password: " pswd

wget --http-user=$name --http-password=$pswd https://nextcloud.robotics.caltech.edu/remote.php/webdav/SubT_Data/Object_Detection/All_data/jan_2021_dataset/training_and_validation_set_with_external.zip
wget --http-user=$name --http-password=$pswd https://nextcloud.robotics.caltech.edu/remote.php/webdav/SubT_Data/Object_Detection/All_data/jan_2021_dataset/test_set.zip

