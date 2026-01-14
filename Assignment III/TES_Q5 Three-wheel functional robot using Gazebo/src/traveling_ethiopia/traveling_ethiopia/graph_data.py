"""
Graph data structure for the Traveling Ethiopia problem.
City coordinates mapped accurately from Figure 1 state space graph.
"""

# City coordinates (mapped from Figure 1 visual layout)
CITY_COORDINATES = {
    # Far left
    'Gambela': (-12.0, -1.0),
    
    # Left column
    'Dembi_Dollo': (-10.0, 1.5),
    'Gore': (-9.0, -0.5),
    'Tepi': (-8.5, -2.0),
    'Mezan_Teferi': (-8.0, -3.5),
    
    # Left-center column
    'Gimbi': (-7.5, 2.0),
    'Nekemte': (-6.5, 2.0),
    'Bedelle': (-6.5, 0.5),
    'Jimma': (-5.5, -1.0),
    'Bonga': (-6.0, -2.0),
    'Dawro': (-5.0, -3.0),
    
    # Center-left column
    'Wolkite': (-3.5, -0.5),
    'Hossana': (-4.0, -2.0),
    'Wolait_Sodo': (-4.5, -3.0),
    'Dilla': (-3.0, -4.5),
    
    # Center column
    'Ambo': (-4.0, 1.5),
    'Addis_Ababa': (-2.0, 1.5),
    'Debra_Birhan': (-1.0, 3.5),
    'Buta_Jirra': (-2.5, -0.5),
    'Worabe': (-2.0, -1.5),
    'Hawassa': (-2.5, -3.0),
    
    # Center-right column  
    'Adama': (0.0, 1.0),
    'Batu': (-0.5, -0.5),
    'Shashemene': (-0.5, -2.5),
    
    # Right-center column
    'Matahara': (1.5, 0.5),
    'Assella': (1.0, -1.0),
    'Assasa': (2.0, -2.0),
    'Dodola': (1.5, -3.5),
    
    # Right column
    'Awash': (3.5, 1.5),
    'Chiro': (5.0, 2.0),
    'Dire_Dawa': (6.5, 2.0),
    
    # Far right upper
    'Harar': (8.0, 1.5),
    'Babile': (9.5, 0.5),
    'Jijiga': (11.0, 1.0),
    
    # Far right lower
    'Dega_Habur': (11.0, -1.5),
    'Kebri_Dehar': (11.5, -3.0),
    'Gode': (10.0, -4.5),
    
    # Lower center-right
    'Bale': (3.5, -3.5),
    'Goba': (5.0, -2.5),
    'Sof_Oumer': (7.0, -3.0),
}

# Graph adjacency list (connections between cities from Figure 1)
CITY_GRAPH = {
    'Gambela': ['Dembi_Dollo', 'Gore'],
    'Dembi_Dollo': ['Gambela', 'Gimbi', 'Gore'],
    'Gore': ['Gambela', 'Dembi_Dollo', 'Tepi', 'Bedelle', 'Jimma'],
    'Gimbi': ['Dembi_Dollo', 'Nekemte'],
    'Tepi': ['Gore', 'Mezan_Teferi', 'Bonga'],
    'Mezan_Teferi': ['Tepi'],
    
    'Nekemte': ['Gimbi', 'Bedelle', 'Ambo'],
    'Bedelle': ['Nekemte', 'Gore', 'Jimma'],
    'Jimma': ['Gore', 'Bedelle', 'Wolkite', 'Hossana', 'Bonga'],
    'Bonga': ['Tepi', 'Jimma', 'Dawro'],
    'Dawro': ['Bonga', 'Wolait_Sodo'],
    'Wolait_Sodo': ['Dawro', 'Hossana', 'Hawassa'],
    'Hossana': ['Jimma', 'Wolait_Sodo', 'Worabe'],
    
    'Ambo': ['Nekemte', 'Addis_Ababa'],
    'Wolkite': ['Jimma', 'Buta_Jirra'],
    'Worabe': ['Hossana', 'Shashemene', 'Hawassa'],
    'Hawassa': ['Wolait_Sodo', 'Worabe', 'Dilla'],
    'Dilla': ['Hawassa'],
    
    'Addis_Ababa': ['Ambo', 'Debra_Birhan', 'Adama', 'Buta_Jirra'],
    'Debra_Birhan': ['Addis_Ababa'],
    'Buta_Jirra': ['Addis_Ababa', 'Wolkite', 'Batu'],
    'Adama': ['Addis_Ababa', 'Matahara'],
    'Batu': ['Buta_Jirra', 'Shashemene'],
    'Shashemene': ['Batu', 'Worabe', 'Assella', 'Dodola'],
    
    'Matahara': ['Adama', 'Awash'],
    'Assella': ['Shashemene', 'Assasa'],
    'Assasa': ['Assella', 'Dodola'],
    'Dodola': ['Shashemene', 'Assasa', 'Bale'],
    'Bale': ['Dodola', 'Goba', 'Sof_Oumer'],
    
    'Awash': ['Matahara', 'Chiro'],
    'Chiro': ['Awash', 'Dire_Dawa'],
    'Dire_Dawa': ['Chiro', 'Harar'],
    'Harar': ['Dire_Dawa', 'Babile'],
    'Babile': ['Harar', 'Jijiga'],
    'Jijiga': ['Babile', 'Dega_Habur'],
    
    'Dega_Habur': ['Jijiga', 'Kebri_Dehar'],
    'Goba': ['Bale', 'Sof_Oumer'],
    'Sof_Oumer': ['Bale', 'Goba', 'Kebri_Dehar'],
    'Kebri_Dehar': ['Dega_Habur', 'Sof_Oumer', 'Gode'],
    'Gode': ['Kebri_Dehar'],
}

def get_neighbors(city):
    """Get neighboring cities for a given city."""
    return CITY_GRAPH.get(city, [])

def get_position(city):
    """Get (x, y) coordinates for a city."""
    return CITY_COORDINATES.get(city, (0, 0))

def get_all_cities():
    """Get list of all city names."""
    return list(CITY_COORDINATES.keys())
