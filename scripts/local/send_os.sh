

# ./scripts/local/get_IdefX_ip.sh # updates the IdefX ip
raspberry_address=$(cat build/IdefX_ip.txt)

echo "Sending everything to the robot dog (pi@$raspberry_address) :"


folder_paths=( "scripts/dog/" "dog_control/controllers/" "dog_control/Idef_OS_X/" "requirements/")
base_path="/home/pi/workspace/dog_control/"

for folder_path in "${folder_paths[@]}"
do
    echo "sending $folder_path..."
	full_path="$base_path$folder_path"
    rsync -r --delete --rsync-path="mkdir -p $full_path && rsync" --exclude __pycache__/ $folder_path pi@$raspberry_address:$full_path
done

rsync setup.py pi@$raspberry_address:"$base_path"setup.py

echo "Finished sending !"
