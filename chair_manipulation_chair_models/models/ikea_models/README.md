# Gazebo models for IKEA dataset

Download ikea dataset from ikea.csail.mit.edu. Create a soft link meshes to the dataset directory.

From one directory level above running

```
pip install -r pip-requirements.txt
python ikea_models/create_models.py
```

Add the above directory and this directory to GAZEBO_MODEL_PATH
TODO Fix this confusion. Make ikea_models a separate repository.

```
gazebo --verbose ikea.world
```

![Colliding ikea world](./media/ikea.world.gif)
![Spread ikea world](./media/ikea.world-spreadout.gif)

