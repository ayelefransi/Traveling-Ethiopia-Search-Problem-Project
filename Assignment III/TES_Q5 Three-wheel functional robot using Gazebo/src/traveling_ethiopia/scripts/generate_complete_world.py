#!/usr/bin/env python3
"""
Generate complete enhanced Ethiopia world file with all cities, robot, and landmarks.
"""

# City data: (name, x, y, color_rgb, size_multiplier)
CITIES = [
    # Left side - Red
    ('Gambela', -10.0, 0.0, (1, 0, 0), 1.0),
    ('Dembi_Dollo', -8.0, 2.0, (1, 0, 0), 1.0),
    ('Gore', -7.0, 0.5, (1, 0, 0), 1.0),
    ('Gimbi', -6.0, 2.5, (1, 0, 0), 1.0),
    ('Tepi', -6.5, -0.5, (1, 0, 0), 1.0),
    ('Mezan_Teferi', -6.0, -1.5, (1, 0, 0), 1.0),
    
    # Central-left - Green
    ('Nekemte', -4.0, 2.5, (0, 1, 0), 1.0),
    ('Bedelle', -4.0, 1.5, (0, 1, 0), 1.0),
    ('Jimma', -3.5, 0.0, (0, 1, 0), 1.2),
    ('Bonga', -4.5, -0.5, (0, 1, 0), 1.0),
    ('Dawro', -3.5, -1.5, (0, 1, 0), 1.0),
    ('Wolait_Sodo', -2.5, -1.5, (0, 1, 0), 1.0),
    ('Hossana', -2.0, -0.5, (0, 1, 0), 1.0),
    
    # Central - Blue
    ('Ambo', -1.5, 2.5, (0, 0, 1), 1.0),
    ('Wolkite', -1.0, 0.5, (0, 0, 1), 1.0),
    ('Worabe', -0.5, -0.5, (0, 0, 1), 1.0),
    ('Hawassa', -1.0, -1.5, (0, 0, 1), 1.2),
    ('Dilla', -0.5, -2.5, (0, 0, 1), 1.0),
    
    # Capital region - Yellow/Gold
    ('Addis_Ababa', 0.5, 2.0, (1, 0.84, 0), 1.5),  # Capital - larger
    ('Debra_Birhan', 1.5, 3.5, (1, 1, 0), 1.0),
    ('Buta_Jirra', 0.0, 0.5, (1, 1, 0), 1.0),
    ('Adama', 2.0, 1.5, (1, 1, 0), 1.1),
    ('Batu', 1.0, 0.0, (1, 1, 0), 1.0),
    ('Shashemene', 1.5, -0.5, (1, 1, 0), 1.0),
    
    # Central-east - Magenta
    ('Matahara', 3.5, 1.5, (1, 0, 1), 1.0),
    ('Assella', 2.5, 0.0, (1, 0, 1), 1.0),
    ('Assasa', 3.0, -0.5, (1, 0, 1), 1.0),
    ('Dodola', 2.5, -1.5, (1, 0, 1), 1.0),
    ('Bale', 3.5, -1.5, (1, 0, 1), 1.0),
    
    # Eastern - Cyan
    ('Awash', 5.0, 2.0, (0, 1, 1), 1.0),
    ('Chiro', 6.0, 2.5, (0, 1, 1), 1.0),
    ('Dire_Dawa', 7.0, 2.5, (0, 1, 1), 1.2),
    ('Harar', 8.0, 2.0, (0, 1, 1), 1.3),  # Historic city - larger
    ('Babile', 9.0, 1.0, (0, 1, 1), 1.0),
    ('Jijiga', 10.0, 1.5, (0, 1, 1), 1.1),
    
    # Far regions - Gray/White
    ('Dega_Habur', 10.0, 0.0, (0.7, 0.7, 0.7), 1.0),
    ('Goba', 5.5, -1.0, (0.7, 0.7, 0.7), 1.0),
    ('Sof_Oumer', 6.5, -1.5, (0.7, 0.7, 0.7), 1.0),
    ('Kebri_Dehar', 10.0, -1.5, (0.7, 0.7, 0.7), 1.0),
    ('Gode', 9.0, -2.5, (0.7, 0.7, 0.7), 1.0),
]

def generate_city_marker(name, x, y, color, size_mult=1.0):
    """Generate SDF for an enhanced city marker."""
    r, g, b = color
    pillar_height = 0.6 * size_mult
    pillar_radius = 0.08 * size_mult
    sphere_radius = 0.15 * size_mult
    sphere_z = pillar_height + sphere_radius + 0.1
    nameplate_z = sphere_z + sphere_radius + 0.15
    emissive = f"{r*0.3} {g*0.3} {b*0.3}"
    
    return f'''    <model name="{name}">
      <pose>{x} {y} 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="pillar">
          <pose>0 0 {pillar_height/2} 0 0 0</pose>
          <geometry>
            <cylinder><radius>{pillar_radius}</radius><length>{pillar_height}</length></cylinder>
          </geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
        <visual name="marker">
          <pose>0 0 {sphere_z} 0 0 0</pose>
          <geometry>
            <sphere><radius>{sphere_radius}</radius></sphere>
          </geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <emissive>{emissive} 1</emissive>
          </material>
        </visual>
        <visual name="nameplate">
          <pose>0 0 {nameplate_z} 0 0 0</pose>
          <geometry>
            <box><size>{0.8*size_mult} {0.3*size_mult} 0.05</size></box>
          </geometry>
          <material>
            <ambient>0.95 0.95 0.95 1</ambient>
            <diffuse>0.95 0.95 0.95 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

'''

# Complete world file
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

    <!-- Ethiopian Sun -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 0.95 0.8 1</diffuse>
      <specular>0.3 0.3 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ethiopian Highlands Terrain -->
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
            <ambient>0.65 0.50 0.35 1</ambient>
            <diffuse>0.70 0.55 0.38 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Simien Mountains (Northwest landmark) -->
    <model name="simien_mountains">
      <pose>-8 4 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="peak1">
          <pose>0 0 1.5 0 0 0</pose>
          <geometry>
            <cone><radius>1.2</radius><length>3.0</length></cone>
          </geometry>
          <material>
            <ambient>0.4 0.35 0.3 1</ambient>
            <diffuse>0.45 0.4 0.35 1</diffuse>
          </material>
        </visual>
        <visual name="peak2">
          <pose>1.5 0.5 1.2 0 0 0</pose>
          <geometry>
            <cone><radius>0.9</radius><length>2.4</length></cone>
          </geometry>
          <material>
            <ambient>0.4 0.35 0.3 1</ambient>
            <diffuse>0.45 0.4 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Lake Tana (Northwest) -->
    <model name="lake_tana">
      <pose>-5 4.5 0.01 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="water">
          <geometry>
            <cylinder><radius>1.5</radius><length>0.02</length></cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.4 0.7 0.8</ambient>
            <diffuse>0.3 0.5 0.8 0.8</diffuse>
            <emissive>0.1 0.2 0.3 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- Bale Mountains (Southeast) -->
    <model name="bale_mountains">
      <pose>4 -2 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="peak">
          <pose>0 0 1.8 0 0 0</pose>
          <geometry>
            <cone><radius>1.5</radius><length>3.6</length></cone>
          </geometry>
          <material>
            <ambient>0.35 0.3 0.25 1</ambient>
            <diffuse>0.4 0.35 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Rift Valley Lakes (Central-South) -->
    <model name="rift_valley_lakes">
      <pose>0 -2 0.01 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="lake1">
          <pose>0 0 0 0 0 0</pose>
          <geometry>
            <cylinder><radius>0.8</radius><length>0.02</length></cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.4 0.7 0.8</ambient>
            <diffuse>0.3 0.5 0.8 0.8</diffuse>
            <emissive>0.1 0.2 0.3 1</emissive>
          </material>
        </visual>
        <visual name="lake2">
          <pose>1.2 -0.5 0 0 0 0</pose>
          <geometry>
            <cylinder><radius>0.6</radius><length>0.02</length></cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.4 0.7 0.8</ambient>
            <diffuse>0.3 0.5 0.8 0.8</diffuse>
            <emissive>0.1 0.2 0.3 1</emissive>
          </material>
        </visual>
      </link>
    </model>

'''

# Add all cities
for city_data in CITIES:
    world_content += generate_city_marker(*city_data)

# Add robot model
world_content += '''
    <!-- Three-Wheeled Robot at Addis Ababa -->
    <model name="three_wheeled_robot">
      <pose>0.5 2.0 0.15 0 0 0</pose>
      
      <link name="base_link">
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.1</iyy><iyz>0</iyz><izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.1</length></cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.1</length></cylinder>
          </geometry>
        </collision>
        <sensor name="imu_sensor" type="imu">
          <topic>imu</topic>
          <update_rate>100</update_rate>
          <always_on>1</always_on>
        </sensor>
      </link>

      <link name="caster_wheel">
        <pose relative_to="base_link">0.15 0 -0.05 0 0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.05</radius></sphere>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.05</radius></sphere>
          </geometry>
        </collision>
      </link>

      <joint name="caster_joint" type="fixed">
        <parent>base_link</parent>
        <child>caster_wheel</child>
      </joint>

      <link name="left_wheel">
        <pose relative_to="base_link">-0.1 0.22 -0.05 -1.5707 0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
          </geometry>
        </collision>
      </link>

      <joint name="left_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>left_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>

      <link name="right_wheel">
        <pose relative_to="base_link">-0.1 -0.22 -0.05 -1.5707 0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.05</length></cylinder>
          </geometry>
        </collision>
      </link>

      <joint name="right_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>

      <link name="lidar_link">
        <pose relative_to="base_link">0 0 0.07 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.03</radius><length>0.04</length></cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <sensor name="lidar" type="gpu_lidar">
          <topic>lidar</topic>
          <update_rate>10</update_rate>
          <lidar>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.1</min>
              <max>10.0</max>
            </range>
          </lidar>
          <always_on>1</always_on>
          <visualize>true</visualize>
        </sensor>
      </link>

      <joint name="lidar_joint" type="fixed">
        <parent>base_link</parent>
        <child>lidar_link</child>
      </joint>

      <link name="camera_link">
        <pose relative_to="base_link">0.2 0 0.05 0 0 0</pose>
        <inertial>
          <mass>0.05</mass>
          <inertia>
            <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box><size>0.02 0.05 0.02</size></box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <sensor name="camera" type="camera">
          <topic>camera</topic>
          <update_rate>30</update_rate>
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <visualize>true</visualize>
        </sensor>
      </link>

      <joint name="camera_joint" type="fixed">
        <parent>base_link</parent>
        <child>camera_link</child>
      </joint>

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
