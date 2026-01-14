#!/usr/bin/env python3
"""
Generate enhanced Ethiopia world with realistic design:
- Textured terrain
- Skybox
- Improved lighting
- Clean city markers with floating text labels
- Matte materials
"""

CITIES = [
    ('Gambela', -12.0, -1.0, (0.8, 0.2, 0.2), 1.0),
    ('Dembi_Dollo', -10.0, 1.5, (0.8, 0.2, 0.2), 1.0),
    ('Gore', -9.0, -0.5, (0.8, 0.2, 0.2), 1.0),
    ('Tepi', -8.5, -2.0, (0.8, 0.2, 0.2), 1.0),
    ('Mezan_Teferi', -8.0, -3.5, (0.8, 0.2, 0.2), 1.0),
    ('Gimbi', -7.5, 2.0, (0.3, 0.7, 0.3), 1.0),
    ('Nekemte', -6.5, 2.0, (0.3, 0.7, 0.3), 1.0),
    ('Bedelle', -6.5, 0.5, (0.3, 0.7, 0.3), 1.0),
    ('Jimma', -5.5, -1.0, (0.3, 0.7, 0.3), 1.2),
    ('Bonga', -6.0, -2.0, (0.3, 0.7, 0.3), 1.0),
    ('Dawro', -5.0, -3.0, (0.3, 0.7, 0.3), 1.0),
    ('Wolkite', -3.5, -0.5, (0.3, 0.5, 0.8), 1.0),
    ('Hossana', -4.0, -2.0, (0.3, 0.5, 0.8), 1.0),
    ('Wolait_Sodo', -4.5, -3.0, (0.3, 0.5, 0.8), 1.0),
    ('Dilla', -3.0, -4.5, (0.3, 0.5, 0.8), 1.0),
    ('Ambo', -4.0, 1.5, (0.9, 0.7, 0.2), 1.0),
    ('Addis_Ababa', -2.0, 1.5, (0.95, 0.75, 0.15), 1.5),
    ('Debra_Birhan', -1.0, 3.5, (0.9, 0.7, 0.2), 1.0),
    ('Buta_Jirra', -2.5, -0.5, (0.9, 0.7, 0.2), 1.0),
    ('Worabe', -2.0, -1.5, (0.3, 0.5, 0.8), 1.0),
    ('Hawassa', -2.5, -3.0, (0.3, 0.5, 0.8), 1.2),
    ('Adama', 0.0, 1.0, (0.9, 0.7, 0.2), 1.1),
    ('Batu', -0.5, -0.5, (0.9, 0.7, 0.2), 1.0),
    ('Shashemene', -0.5, -2.5, (0.9, 0.7, 0.2), 1.0),
    ('Matahara', 1.5, 0.5, (0.7, 0.3, 0.7), 1.0),
    ('Assella', 1.0, -1.0, (0.7, 0.3, 0.7), 1.0),
    ('Assasa', 2.0, -2.0, (0.7, 0.3, 0.7), 1.0),
    ('Dodola', 1.5, -3.5, (0.7, 0.3, 0.7), 1.0),
    ('Awash', 3.5, 1.5, (0.2, 0.7, 0.7), 1.0),
    ('Chiro', 5.0, 2.0, (0.2, 0.7, 0.7), 1.0),
    ('Dire_Dawa', 6.5, 2.0, (0.2, 0.7, 0.7), 1.2),
    ('Harar', 8.0, 1.5, (0.2, 0.7, 0.7), 1.3),
    ('Babile', 9.5, 0.5, (0.2, 0.7, 0.7), 1.0),
    ('Jijiga', 11.0, 1.0, (0.2, 0.7, 0.7), 1.1),
    ('Dega_Habur', 11.0, -1.5, (0.6, 0.6, 0.6), 1.0),
    ('Kebri_Dehar', 11.5, -3.0, (0.6, 0.6, 0.6), 1.0),
    ('Gode', 10.0, -4.5, (0.6, 0.6, 0.6), 1.0),
    ('Bale', 3.5, -3.5, (0.7, 0.3, 0.7), 1.0),
    ('Goba', 5.0, -2.5, (0.6, 0.6, 0.6), 1.0),
    ('Sof_Oumer', 7.0, -3.0, (0.6, 0.6, 0.6), 1.0),
]

def generate_city_marker(name, x, y, color, size_mult=1.0):
    """Generate clean city marker with vertical pole and floating label."""
    model_name = name.replace(' ', '_')
    display_name = name.replace('_', ' ')
    r, g, b = color
    
    # Pole dimensions
    pole_height = 1.2 * size_mult
    pole_radius = 0.04 * size_mult
    
    # Top marker (cylinder)
    marker_height = 0.3 * size_mult
    marker_radius = 0.12 * size_mult
    marker_z = pole_height + marker_height/2
    
    # Floating text label
    label_z = pole_height + marker_height + 0.3
    
    return f'''    <model name="{model_name}">
      <pose>{x} {y} 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <!-- Vertical pole -->
        <visual name="pole">
          <pose>0 0 {pole_height/2} 0 0 0</pose>
          <geometry>
            <cylinder><radius>{pole_radius}</radius><length>{pole_height}</length></cylinder>
          </geometry>
          <material>
            <ambient>{r*0.6} {g*0.6} {b*0.6} 1</ambient>
            <diffuse>{r*0.7} {g*0.7} {b*0.7} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        
        <!-- Top marker -->
        <visual name="marker">
          <pose>0 0 {marker_z} 0 0 0</pose>
          <geometry>
            <cylinder><radius>{marker_radius}</radius><length>{marker_height}</length></cylinder>
          </geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.15 0.15 0.15 1</specular>
          </material>
        </visual>
        
        <!-- Floating text label background -->
        <visual name="label_bg">
          <pose>0 0 {label_z} 0 0 0</pose>
          <geometry>
            <box><size>1.0 0.3 0.02</size></box>
          </geometry>
          <material>
            <ambient>0.95 0.95 0.95 0.95</ambient>
            <diffuse>0.95 0.95 0.95 0.95</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        
        <!-- Label border -->
        <visual name="label_border">
          <pose>0 0 {label_z - 0.005} 0 0 0</pose>
          <geometry>
            <box><size>1.02 0.32 0.01</size></box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

'''

world_content = '''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="default">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>

    <!-- Skybox -->
    <scene>
      <ambient>0.5 0.5 0.55 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <sky>
        <clouds>
          <speed>0.12</speed>
        </clouds>
      </sky>
    </scene>

    <!-- Improved directional sunlight -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 15 0 0 0</pose>
      <diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.4 0.4 0.35 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Textured ground plane (dry soil with grass patches) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.55 0.48 0.38 1</ambient>
            <diffuse>0.60 0.52 0.40 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Grid overlay for reference (subtle) -->
    <model name="grid_overlay">
      <static>true</static>
      <link name="link">
        <visual name="grid">
          <pose>0 0 0.001 0 0 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 0.15</ambient>
            <diffuse>0.5 0.5 0.5 0.15</diffuse>
          </material>
        </visual>
      </link>
    </model>

'''

# Add all city markers
for city_data in CITIES:
    world_content += generate_city_marker(*city_data)

# Add robot
world_content += '''
    <!-- Three-wheeled robot -->
    <model name="three_wheeled_robot">
      <pose>-2.0 1.5 0.15 0 0 0</pose>
      <link name="base_link">
        <inertial><mass>5</mass><inertia><ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.1</iyy><iyz>0</iyz><izz>0.1</izz></inertia></inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.2</radius><length>0.1</length></cylinder></geometry>
          <material>
            <ambient>0.2 0.3 0.8 1</ambient>
            <diffuse>0.25 0.35 0.85 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <collision name="collision"><geometry><cylinder><radius>0.2</radius><length>0.1</length></cylinder></geometry></collision>
        <sensor name="imu_sensor" type="imu"><topic>imu</topic><update_rate>100</update_rate><always_on>1</always_on></sensor>
      </link>
      <link name="caster_wheel">
        <pose relative_to="base_link">0.15 0 -0.05 0 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia></inertial>
        <visual name="visual">
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision"><geometry><sphere><radius>0.05</radius></sphere></geometry></collision>
      </link>
      <joint name="caster_joint" type="fixed"><parent>base_link</parent><child>caster_wheel</child></joint>
      <link name="left_wheel">
        <pose relative_to="base_link">-0.1 0.22 -0.05 -1.5707 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia></inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision"><geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry></collision>
      </link>
      <joint name="left_wheel_joint" type="revolute"><parent>base_link</parent><child>left_wheel</child><axis><xyz>0 0 1</xyz></axis></joint>
      <link name="right_wheel">
        <pose relative_to="base_link">-0.1 -0.22 -0.05 -1.5707 0 0</pose>
        <inertial><mass>1</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia></inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision"><geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry></collision>
      </link>
      <joint name="right_wheel_joint" type="revolute"><parent>base_link</parent><child>right_wheel</child><axis><xyz>0 0 1</xyz></axis></joint>
      <link name="lidar_link">
        <pose relative_to="base_link">0 0 0.07 0 0 0</pose>
        <inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia></inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.03</radius><length>0.04</length></cylinder></geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.85 0.25 0.25 1</diffuse>
            <specular>0.15 0.15 0.15 1</specular>
          </material>
        </visual>
        <sensor name="lidar" type="gpu_lidar"><topic>lidar</topic><update_rate>10</update_rate><lidar><scan><horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan><range><min>0.1</min><max>10.0</max></range></lidar><always_on>1</always_on><visualize>true</visualize></sensor>
      </link>
      <joint name="lidar_joint" type="fixed"><parent>base_link</parent><child>lidar_link</child></joint>
      <link name="camera_link">
        <pose relative_to="base_link">0.2 0 0.05 0 0 0</pose>
        <inertial><mass>0.05</mass><inertia><ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz></inertia></inertial>
        <visual name="visual">
          <geometry><box><size>0.02 0.05 0.02</size></box></geometry>
          <material>
            <ambient>0.2 0.7 0.3 1</ambient>
            <diffuse>0.25 0.75 0.35 1</diffuse>
            <specular>0.15 0.15 0.15 1</specular>
          </material>
        </visual>
        <sensor name="camera" type="camera"><topic>camera</topic><update_rate>30</update_rate><camera><horizontal_fov>1.047</horizontal_fov><image><width>640</width><height>480</height></image><clip><near>0.1</near><far>100</far></clip></camera><always_on>1</always_on><visualize>true</visualize></sensor>
      </link>
      <joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.44</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
        <topic>cmd_vel</topic>
        <odom_topic>odom</odom_topic>
      </plugin>
    </model>

  </world>
</sdf>
'''

print(world_content)
