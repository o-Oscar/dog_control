

sudo apt-get install python3.9-dev
python3.9 -m venv .venv

# add the mujoco specific stuff to the env activate
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/home/oscar/.mujoco/mujoco210/bin" >> .venv/bin/activate
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/lib/nvidia" >> .venv/bin/activate
echo "export LD_PRELOAD=\$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libGLEW.so" >> .venv/bin/activate


source .venv/bin/activate
which python

pip install -r requirements/local.txt

pip install -e ./