

# ./scripts/local/get_IdefX_ip.sh # updates the IdefX ip
raspberry_address=$(cat build/IdefX_ip.txt)

echo "Retrieving the logs of the robot dog (pi@$raspberry_address) :"


base_path="/home/pi/workspace/dog_control/"
rsync -r --delete pi@$raspberry_address:"$base_path"logs/local/ logs/dog

echo "Finished retrieving !"
