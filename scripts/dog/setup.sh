

python3.7 -m venv .venv

echo "./home/pi/workspace/dog_control/scripts/dog/activate_can.sh" >> .venv/bin/activate

source .venv/bin/activate
which python

pip install --upgrade pip

pip install -r requirements/dog.txt

pip install -e ./


mkdir data