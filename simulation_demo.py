import pygame
import heapq
import numpy as np
CIG = 16  # Cells in Grid
PPC = 40  # Pixels per Cell

pygame.init()
screen = pygame.display.set_mode(((CIG * PPC),(CIG * PPC)))
pygame.display.set_caption("Drone Mapping and Pathfinding")
grid = [[0 for _ in range(CIG)] for _ in range(CIG)] 


clock = pygame.time.Clock()         

############################################################## Map Generation ###############################################################

frames_per_second = 10  
# Movement (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
frames_per_second = 10  

#Zigzag animation to simulate drone scanning the floor
Zig_Zag = [(x, y) for y in range(CIG)
    for x in (range(CIG) if y % 2 == 0 else reversed(range(CIG)))]


#Generates a random maze with a ratio of 4:1 white to black squares
def simulate_floor_scan():
    global grid
    for step_x, step_y in Zig_Zag:
        color = 1 if np.random.rand() > 0.2 else -1 
        grid[step_y][step_x] = color
        print(f"Simulated move to ({step_x}, {step_y})")

        #update grid every step
        draw_grid()
        pygame.display.flip()
        clock.tick(frames_per_second)



############################################################## Path Generation ###############################################################


start = None
end = None

# clours int the grid squares with appropriate colour
def draw_grid():
    for y in range(CIG):
        for x in range(CIG):
                color = (150, 0, 100)        # purple square (unmapped regions)
                if   grid[y][x] ==  1:
                     color = (255, 255, 255) # white square (traversable path)
                elif grid[y][x] == -1:
                     color = (50, 50, 50)    # black square (obstacle)
                elif grid[y][x] ==  3:
                     color = (255, 255, 0)    # yellow square (explored path)
                elif grid[y][x] ==  2:
                     color = (0, 255, 0)      # green square (final path)

                # Make cell
                pygame.draw.rect(screen,color,(x * PPC, y * PPC, PPC, PPC),)

                # Make grid
                pygame.draw.rect(screen,(50, 50, 50),(x * PPC, y * PPC, PPC, PPC),1,)

    # Draw start point 
    if start:
        sx,sy = start
        pygame.draw.circle(screen, (0, 0, 255) , (sx * PPC + PPC // 2, sy * PPC + PPC // 2), PPC // 4) # blue dot (start point)

    # Draw end point 
    if end:
        ex,ey = end
        pygame.draw.circle(screen, (255, 0, 0), (ex * PPC + PPC // 2, ey * PPC + PPC // 2), PPC // 4) # red dot  (end point)
frames_per_second = 10  

def a_star_search(maze, start, end):
 # Heuristic function which prioritise movinging in the x-axis then the y-axis (seems to give better results than when axis are treated as equal)
    def heuristic_function(a, b):
        return abs(a[0] - b[0]) * 2 + abs(a[1] - b[1])

       
    if not start or not end:
        print("Error: Start or End point is None")
        return None

    explored = {}
    not_explored = []
    heapq.heappush(not_explored, (0, start)) # Add the start point into priority queue (start has 0 priority)
    cost = {start: 0}                        # tracks total cost of reaching a given square


    while not_explored:
        # Pop the node with the lowest priority (f-score)
        current_priority, current_position = heapq.heappop(not_explored)
  
        x, y = current_position

        # Mark the node as explored
        if grid[y][x] != 2:  # Skip final path marking (to avoid overwriting)
            grid[y][x] = 3  # Mark as explored (yellow)
        draw_grid()
        pygame.display.flip()
        clock.tick(frames_per_second)

        # If goal is reached, backtrack to build the path
        if current_position == end:
            path = []
            while current_position in explored:
                path.append(current_position)
                current_position = explored[current_position]
            path.append(start)
            path.reverse()
            for px, py in path:
                grid[py][px] = 2
                draw_grid()
                pygame.display.flip()
                clock.tick(frames_per_second)
            return path

        # Explore neighbors
        for dx, dy in DIRECTIONS:
            neighbor = (x + dx, y + dy)
            nx, ny = neighbor

            if (0 <= nx < CIG and 0 <= ny < CIG and maze[ny][nx] == 1):  # Ensure neighbor is valid and walkable
                cost_accum = cost[current_position] + 1  # Each square has a cost of 1

                if neighbor not in cost or cost_accum < cost[neighbor]:
                    explored[neighbor] = current_position
                    cost[neighbor] = cost_accum
                    f_score = cost_accum + heuristic_function(neighbor, end)  # f = g + h
                    heapq.heappush(not_explored, (f_score, neighbor))

    # If we exhaust the queue without finding a path, return None
    return None

def prepare_maze(grid):
    return [[1 if cell == 1 else 0 for cell in row] for row in grid]


############################################################## Main #################################################################



def main():
    simulate_floor_scan()
    global start, end
    running = True

    while running:
        screen.fill((0, 0, 0))  # Clear the screen

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                # Get mouse position and map to grid coordinates
                mouse_x, mouse_y = pygame.mouse.get_pos()
                grid_x, grid_y = mouse_x // PPC, mouse_y // PPC

                # Allow the user to click to set start and end points
                if not start:
                    start = (grid_x, grid_y)  # Set start point
                elif not end:
                    end = (grid_x, grid_y)  # Set end point

        draw_grid()
        pygame.display.flip()
        clock.tick(frames_per_second)

        if start and end:
            # If both start and end points are set, break out of the loop
            running = False

    maze = prepare_maze(grid)

    print(f"Finding path from {start} to {end}...")
    path = a_star_search(maze, start, end)
    if path:
        print(f"Path found: {path}")
    else:
        print("No path found!")

    pygame.image.save(screen, "final_path.png")
    print("Final path saved as 'final_path.png'.")

    # Keep the window open after pathfinding
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

if __name__ == "__main__":
    main()
