#!/usr/bin/env python3
"""
Script to add text labels above each city marker in the Ethiopia world file.
This adds visual text elements that display city names in Gazebo.
"""

# City data with positions and colors
cities_data = [
    # (name, x, y, color_group)
    # Left side cities - Red
    ('Gambela', -10.0, 0.0, 'red'),
    ('Dembi_Dollo', -8.0, 2.0, 'red'),
    ('Gore', -7.0, 0.5, 'red'),
    ('Gimbi', -6.0, 2.5, 'red'),
    ('Tepi', -6.5, -0.5, 'red'),
    ('Mezan_Teferi', -6.0, -1.5, 'red'),
    
    # Central-left cities - Green
    ('Nekemte', -4.0, 2.5, 'green'),
    ('Bedelle', -4.0, 1.5, 'green'),
    ('Jimma', -3.5, 0.0, 'green'),
    ('Bonga', -4.5, -0.5, 'green'),
    ('Dawro', -3.5, -1.5, 'green'),
    ('Wolait_Sodo', -2.5, -1.5, 'green'),
    ('Hossana', -2.0, -0.5, 'green'),
    
    # Central cities - Blue
    ('Ambo', -1.5, 2.5, 'blue'),
    ('Wolkite', -1.0, 0.5, 'blue'),
    ('Worabe', -0.5, -0.5, 'blue'),
    ('Hawassa', -1.0, -1.5, 'blue'),
    ('Dilla', -0.5, -2.5, 'blue'),
    
    # Capital region - Yellow
    ('Addis_Ababa', 0.5, 2.0, 'yellow'),
    ('Debra_Birhan', 1.5, 3.5, 'yellow'),
    ('Buta_Jirra', 0.0, 0.5, 'yellow'),
    ('Adama', 2.0, 1.5, 'yellow'),
    ('Batu', 1.0, 0.0, 'yellow'),
    ('Shashemene', 1.5, -0.5, 'yellow'),
    
    # Central-east - Magenta
    ('Matahara', 3.5, 1.5, 'magenta'),
    ('Assella', 2.5, 0.0, 'magenta'),
    ('Assasa', 3.0, -0.5, 'magenta'),
    ('Dodola', 2.5, -1.5, 'magenta'),
    ('Bale', 3.5, -1.5, 'magenta'),
    
    # Eastern - Cyan
    ('Awash', 5.0, 2.0, 'cyan'),
    ('Chiro', 6.0, 2.5, 'cyan'),
    ('Dire_Dawa', 7.0, 2.5, 'cyan'),
    ('Harar', 8.0, 2.0, 'cyan'),
    ('Babile', 9.0, 1.0, 'cyan'),
    ('Jijiga', 10.0, 1.5, 'cyan'),
    
    # Far regions - Gray
    ('Dega_Habur', 10.0, 0.0, 'gray'),
    ('Goba', 5.5, -1.0, 'gray'),
    ('Sof_Oumer', 6.5, -1.5, 'gray'),
    ('Kebri_Dehar', 10.0, -1.5, 'gray'),
    ('Gode', 9.0, -2.5, 'gray'),
]

def generate_text_label(city_name, x, y):
    """Generate SDF for a text label above a city."""
    # Replace underscores with spaces for display
    display_name = city_name.replace('_', ' ')
    
    # Text label positioned above the city marker
    label_z = 0.5  # Height above ground
    
    return f'''
    <!-- Text Label for {city_name} -->
    <model name="{city_name}_label">
      <pose>{x} {y} {label_z} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="text">
          <geometry>
            <box><size>0.01 0.01 0.01</size></box>
          </geometry>
          <plugin filename="gz-sim-label-system" name="gz::sim::systems::Label">
            <label>{display_name}</label>
          </plugin>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''

# Generate all text labels
print("<!-- City Name Labels -->")
for city_name, x, y, color in cities_data:
    print(generate_text_label(city_name, x, y))
