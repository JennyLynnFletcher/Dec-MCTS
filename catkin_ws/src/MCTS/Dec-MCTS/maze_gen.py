from time import time
from scipy import sparse
import random


def generate_maze(h, w, seed=None):
    """
    h, w: required maze dimensions, must be odd
    returns:
    maze: scipy.sparse.dok_matrix of shape (h+2, w+2) with contents
        TODO: maybe change this to actually h x w?
    0: wall
    1: traversible
    """
    random.seed(seed)
    if h % 2 and w % 2:
        maze = sparse.dok_matrix((h + 2, w + 2), dtype=int)
        maze[1, 1] = 1
        walls = {(1, 2), (2, 1)}
        while walls:
            new_wall = random.choice(tuple(walls))
            walls.remove(new_wall)
            y, x = new_wall
            if y == 0 or x == 0 or y == h + 1 or x == w + 1:
                continue
            if not y % 2:
                top = (y - 1, x)
                bot = (y + 1, x)
                if maze[top] != maze[bot]:
                    maze[new_wall] = 1
                    if maze[top]:
                        maze[bot] = 1
                        walls.update([(y + 2, x), (y + 1, x - 1), (y + 1, x + 1)])
                    elif maze[bot]:
                        maze[top] = 1
                        walls.update([(y - 2, x), (y - 1, x - 1), (y - 1, x + 1)])
            else:
                left = (y, x - 1)
                right = (y, x + 1)
                if maze[left] != maze[right]:
                    maze[new_wall] = 1
                    if maze[left]:
                        maze[right] = 1
                        walls.update([(y, x + 2), (y - 1, x + 1), (y + 1, x + 1)])
                    elif maze[right]:
                        maze[left] = 1
                        walls.update([(y, x - 2), (y - 1, x - 1), (y + 1, x - 1)])
        return maze
    else:
        raise ValueError("width and height must be odd numbers in order to work properly")


def fill_in_maze(maze):
    """
    maze: sparse scipy matrix, of dimensions 2k+1 x 2l+1 of contents
    0: unknown
    1: traversible
    2: known wall
    works on the matrix IN PLACE
    returns with fully grown maze with
    0: known wall
    1: traversible
    """
    walls = set()
    h, w = maze.shape
    marked = set()

    def bfs(start_point):
        stack = [start_point]
        marked.add(start_point)
        while stack:
            current = stack.pop()
            y, x = current
            for offset in (-1, 1):
                neighbour = (y + offset, x)
                if maze[neighbour] == 1:
                    if neighbour not in marked:
                        stack.append(neighbour)
                        marked.add(neighbour)
                elif maze[neighbour] == 0:
                    walls.add(neighbour)

                neighbour = (y, x + offset)
                if maze[neighbour] == 1:
                    if neighbour not in marked:
                        stack.append(neighbour)
                        marked.add(neighbour)
                elif maze[neighbour] == 0:
                    walls.add(neighbour)

    non_empties = sparse.find(maze)
    bfs_start = None
    wall_paths = []
    for i in range(len(non_empties[0])):
        y, x, val = (arr[i] for arr in non_empties)
        if val == 1:
            bfs_start = (y, x)
            if y % 2 == 0 or x % 2 == 0:
                wall_paths.append(tuple([y, x]))
    if bfs_start is None:
        return ValueError("no known traversible point found in maze")
    for y, x in wall_paths:
        if x % 2:
            maze[y + 1, x] = 1
            maze[y - 1, x] = 1
        else:
            maze[y, x + 1] = 1
            maze[y, x - 1] = 1

    bfs(bfs_start)

    while walls:
        new_wall = random.choice(tuple(walls))
        walls.remove(new_wall)
        y, x = new_wall
        if y == 0 or x == 0 or y == h - 1 or x == w - 1:
            maze[new_wall] = 0
            continue
        elif maze[new_wall] == 2:
            continue
        elif y % 2 == x % 2:
            continue

        if not y % 2:
            top = (y - 1, x)
            bot = (y + 1, x)
            if not(top in marked and bot in marked):
                maze[new_wall] = 1
                if top in marked:
                    if maze[bot] == 1:
                        bfs(bot)
                        continue

                    maze[bot] = 1
                    marked.add(bot)
                    walls.update([(y + 2, x), (y + 1, x - 1), (y + 1, x + 1)])

                elif bot in marked:
                    if maze[top] == 1:
                        bfs(top)
                        continue

                    maze[top] = 1
                    marked.add(top)
                    walls.update([(y - 2, x), (y - 1, x - 1), (y - 1, x + 1)])

        elif not x % 2:
            left = (y, x - 1)
            right = (y, x + 1)
            if not(left in marked and right in marked):
                maze[new_wall] = 1
                if left in marked:
                    if maze[right] == 1:
                        bfs(right)
                        continue

                    maze[right] = 1
                    marked.add(right)
                    walls.update([(y, x + 2), (y - 1, x + 1), (y + 1, x + 1)])

                elif right in marked:
                    if maze[left] == 1:
                        bfs(left)
                        continue

                    maze[left] = 1
                    marked.add(left)
                    walls.update([(y, x - 2), (y - 1, x - 1), (y + 1, x - 1)])

    for i in range(len(non_empties[0])):
        y, x, val = (arr[i] for arr in non_empties)
        if val == 2:
            maze[y, x] = 0

    return maze


if __name__ == '__main__':
    a = sparse.dok_matrix((15, 15), int)
    for i in range(15):
        a[0, i] = 2
        a[14, i] = 2
        a[i, 0] = 2
        a[i, 14] = 2
    a[1, 1] = 1
    a[2, 1] = 1
    a[3, 1] = 1
    a[2, 2] = 2
    a[3, 2] = 1
    from maze import generate_maze as gen_m

    times = [[], []]
    # for _ in range(500):
    start = time()
    maze = gen_m(a.copy(), (5,5))
    mid = time()
    maze.add_robot(0, (1,2))
    maze.add_robot(1, (1,2))
    mid2 = time()
    maze.get_score(agent_obs=a)
    end = time()
    times[0].append(mid - start)
    times[1].append(end-mid2)
    print(maze.walls.todense())

    print(sum(times[0])/len(times[0]), sum(times[1])/len(times[1]))

    # print(fill_in_maze(sparse.vstack((a[:-1,:], a), format="dok")).todense())
    # tall_new = sparse.vstack((a[:-1,:], sparse.dok_matrix((7, 7), dtype=int)), format="dok")
    # print(fill_in_maze(tall_new).todense())
    #
    # double_new = sparse.hstack((tall_new, sparse.dok_matrix((13, 6), dtype=int)), format="dok")
    #
    # print(fill_in_maze(double_new).todense())
