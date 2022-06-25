

# rsync ...

echo "Sending everything to the robot dog..."

raspberry_address="192.168.1.43"


folder_path="scripts/dog/"
full_path="/home/pi/workspace/dog_control/$folder_path"
rsync -r --rsync-path="mkdir -p $full_path && rsync" --exclude __pycache__/ $folder_path pi@192.168.1.43:$full_path

folder_path="dog_control/controllers/"
full_path="/home/pi/workspace/dog_control/$folder_path"
rsync -r --rsync-path="mkdir -p $full_path && rsync" --exclude __pycache__/ $folder_path pi@192.168.1.43:$full_path
 
folder_path="dog_control/Idef_OS_X/"
full_path="/home/pi/workspace/dog_control/$folder_path"
rsync -r --rsync-path="mkdir -p $full_path && rsync" --exclude __pycache__/ $folder_path pi@192.168.1.43:$full_path
 
echo "Finished sending !"
