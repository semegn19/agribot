import random

FILENAME = "ethiopian_farm_generated.sdf"
# Exact coordinates from your previous SDF
Y_ROWS = [6.0, 3.0, -3.0, -6.0]
X_COLS = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
DISEASE_CHANCE = 0.25 

# Textures - Ensure these exist in your textures folder!
HEALTHY_TEX = "textures/healthy_leaf.jpg"
DISEASE_TEX = "textures/tomato_blight.jpg"

def create_plant_model(name, x, y, is_diseased):
    tex = DISEASE_TEX if is_diseased else HEALTHY_TEX
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x} {y} 0.5 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>0.2 0.2 1.0</size></box></geometry>
        </collision>
        <visual name='visual_v'>
          <geometry><box><size>0.01 0.8 1.0</size></box></geometry>
          <material><pbr><metal><albedo_map>{tex}</albedo_map></metal></pbr></material>
        </visual>
        <visual name='visual_h'>
          <pose>0 0 0 0 0 1.57</pose>
          <geometry><box><size>0.01 0.8 1.0</size></box></geometry>
          <material><pbr><metal><albedo_map>{tex}</albedo_map></metal></pbr></material>
        </visual>
      </link>
    </model>"""

# Fixed assets from your previous world
SDF_HEADER = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="agribot_farm">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="farm_sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.3 0.2 0.1 1</ambient><diffuse>0.3 0.2 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="weed_1">
      <static>true</static>
      <link name="link">
        <visual name="v"><pose>3.5 0 0.05 0 0 0</pose><geometry><box><size>0.2 0.2 0.1</size></box></geometry>
        <material><ambient>0.4 0.4 0.1 1</ambient></material></visual>
      </link>
    </model>
    <model name="weed_2">
      <static>true</static>
      <link name="link">
        <visual name="v"><pose>6.0 -1.2 0.05 0 0 0.5</pose><geometry><box><size>0.3 0.15 0.1</size></box></geometry>
        <material><ambient>0.3 0.3 0 1</ambient></material></visual>
      </link>
    </model>
"""

with open(FILENAME, "w") as f:
    f.write(SDF_HEADER)
    
    # Generate Plants at exact grid locations
    for row_idx, y in enumerate(Y_ROWS):
        for col_idx, x in enumerate(X_COLS):
            is_unhealthy = random.random() < DISEASE_CHANCE
            name = f"plant_r{row_idx}_c{col_idx}"
            f.write(create_plant_model(name, x, y, is_unhealthy))
            
    f.write("\n  </world>\n</sdf>")
print(f"Done! {FILENAME} created with your specific layout.")
