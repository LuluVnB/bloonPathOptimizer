import heapq
from math import sqrt
from collections import defaultdict


# 1. Data Structures

class Tower:
    def __init__(self, x, y, tower_type):
        self.x = x
        self.y = y
        self.type = tower_type
        #tower properties based on type
        self.dps = 1 if tower_type == "Dart" else 3 if tower_type == "Bomb" else 2
        self.pierce = 1 if tower_type == "Dart" else 0
        self.can_pop_lead = tower_type in ["Bomb", "Ice"]

class Bloon:
    def __init__(self, bloon_type):
        self.type = bloon_type
        #bloon stats (cost, HP, immunities)
        self.stats = {
            "Red": {"cost": 1, "HP": 1, "immune": []},
            "Lead": {"cost": 90, "HP": 50, "immune": ["sharp"]},
            "Ceramic": {"cost": 120, "HP": 100, "immune": []},
            "MOAB": {"cost": 500, "HP": 500, "immune": []}
        }[bloon_type]

# 2.bloon Ranking Algorithm

def rank_bloons(opponent_towers):
    bloon_types = ["Red", "Lead", "Ceramic", "MOAB"]
    ranked = []
    
    for bloon_type in bloon_types:
        bloon = Bloon(bloon_type)
        threat = sum(
            tower.dps for tower in opponent_towers
            if tower.type not in bloon.stats["immune"]
        )
        effective_hp = bloon.stats["HP"] - threat
        if effective_hp <= 0:
            continue  #skip bloons that would instantly die
        
        score = effective_hp / bloon.stats["cost"]
        ranked.append((bloon_type, score))
    
    return sorted(ranked, key=lambda x: -x[1])  #descending by score


# 3. Modified A* Pathfinding

def a_star_bloon_path(start, end, map_graph, bloon_type, towers):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, end, bloon_type, towers)
    
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == end:
            return reconstruct_path(came_from, current)
        
        for neighbor in map_graph.neighbors(current):
            #dynamic edge cost based on tower threats to this bloon
            edge_cost = 1  #base cost (1 per tile)
            for tower in towers:
                if tower.type not in Bloon(bloon_type).stats["immune"]:
                    dist_to_tower = sqrt((neighbor[0]-tower.x)**2 + (neighbor[1]-tower.y)**2)
                    if dist_to_tower <= 3:  #the tower range
                        edge_cost += tower.dps * 0.5  #the danger penalty
            
            tentative_g = g_score[current] + edge_cost
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, end, bloon_type, towers)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  #no path found

def heuristic(node, end, bloon_type, towers):
    #euclidean distance + danger estimate
    dx = end[0] - node[0]
    dy = end[1] - node[1]
    distance = sqrt(dx*dx + dy*dy)
    
    danger = sum(
        tower.dps for tower in towers
        if tower.type not in Bloon(bloon_type).stats["immune"]
        and sqrt((node[0]-tower.x)**2 + (node[1]-tower.y)**2) <= 3
    )
    return distance + danger * 0.3

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


# 4. Map Graph (Simplified "Shapes" Map)

class MapGraph:
    def __init__(self):
        #grid representation of "Shapes" map
        self.grid = [
            [0, 0, 1, 0, 0],  # 0=path, 1=obstacle
            [0, 1, 0, 0, 1],
            [0, 0, 0, 1, 0],
            [1, 0, 0, 0, 0]
        ]
        self.width = len(self.grid[0])
        self.height = len(self.grid)
    
    def neighbors(self, node):
        x, y = node
        dirs = [(0,1), (1,0), (0,-1), (-1,0)]  # 4-directional
        neighbors = []
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny][nx] == 0:  # Only traverse paths
                    neighbors.append((nx, ny))
        return neighbors


# 5.Full Optimization Pipeline

def optimize_bloon_strategy(opponent_towers, map_graph):
    #Step 1: Rank bloons
    ranked_bloons = rank_bloons(opponent_towers)
    if not ranked_bloons:
        return None, None
    
    best_bloon = ranked_bloons[0][0]
    start = (0, 0)  #Start coordinate
    end = (map_graph.width-1, map_graph.height-1)  # End coordinate
    
    #Step 2: Find best path for this bloon
    path = a_star_bloon_path(start, end, map_graph, best_bloon, opponent_towers)
    
    return best_bloon, path


# 6. Test Case (Shapes Map)

if __name__ == "__main__":
    #opponent's towers
    towers = [
        Tower(2, 1, "Dart"),  #can't pop Lead
        Tower(3, 2, "Bomb")   #pops Lead
    ]
    
    map_graph = MapGraph()
    best_bloon, path = optimize_bloon_strategy(towers, map_graph)
    
    print(f"Best Bloon: {best_bloon}")
    print(f"Optimal Path: {path}")