# Robotsimulator

**Brief instruction:**
- Git clone this repository **<a href="https://gitlab.liu.se/cvl/tsbb18/-/tree/main">tsbb18</a>**
- Setup Python environement for the project
    ```
    #install python venv
    sudo apt install python3.12-venv
    #create python env
    python3 -m venv env-tsbb18-ServoArm
    #activate environment
    source env-TSB18-ServoArm/bin/activate
    #install pip
    sudo apt install python3-pip
    ```

- Install computational dependencies and **<a href="https://pybullet.org/wordpress">Pybullet</a>** with pip.
    ```
    pip install numpy
    pip install opencv-python
    pip install pybullet
    ```

- Run the `client.py` file from the terminal.
    ```
    #run in ../tsbb18/robot_sim/
    python client.py
    ```

**In the 3D window:**

- Rotate using `Ctrl` + the left mouse button + move.
- Pan using `Ctrl` + middle mouse button + move.
- Zoom using the scroll wheel.
