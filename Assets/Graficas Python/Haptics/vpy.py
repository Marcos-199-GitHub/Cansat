from ursina import *

# create a window
app = Ursina()

# most things in ursina are Entities. An Entity is a thing you place in the world.
# you can think of them as GameObjects in Unity or Actors in Unreal.
# the first paramenter tells us the Entity's model will be a 3d-model called 'cube'.
# ursina includes some basic models like 'cube', 'sphere' and 'quad'.

# the next parameter tells us the model's color should be orange.

# 'scale_y=2' tells us how big the entity should be in the vertical axis, how tall it should be.
# in ursina, positive x is right, positive y is up, and positive z is forward.
origin = Entity(model='sphere',color = color.black,scale = (0.1,0.1,0.1))
xaxis = Entity(model = 'cube', color = color.red, scale = (20,0.05,0.05))
yaxis = Entity(model = 'cube', color = color.green, scale = (0.05,20,0.05))
zaxis = Entity(model = 'cube', color = color.blue, scale = (0.05,0.05,20))
player = Entity(model='cube', color=color.orange, scale_y=1)
sun = DirectionalLight()
sun.look_at(Vec3(1,1,1))
Sky()

camera.position = (1,-30,-75)
camera.rotation_x = -20
camera.rotation_z = 5

print(camera.get_position())
print(camera.rotation)
camera.fov = 300
# create a function called 'update'.
# this will automatically get called by the engine every frame.

def update():
    
    player.set_position((1,1,1))
    #player.x -= 1 * time.dt


# this part will make the player move left or right based on our input.
# to check which keys are held down, we can check the held_keys dictionary.
# 0 means not pressed and 1 means pressed.
# time.dt is simply the time since the last frame. by multiplying with this, the
# player will move at the same speed regardless of how fast the game runs.


def input(key):
    if key == 'space':
        player.y += 1
        invoke(setattr, player, 'y', player.y-1, delay=.25)


# start running the game
app.run()