from A_Star import a_star
import timeit

def sense(i, j, grid, dim):
    ans=0
    for I in [i-1,i,i+1]:
        for J in [j-1,j,j+1]:
            if (I,J) == (i,j) or I>=dim or J>=dim or I<0 or J<0:
                continue
            if grid[I][J]==1:
                ans = ans + 1
    return ans

def count_neighbors(i, j, dim):
    if (i==0 and j==0) or (i==dim-1 and j==0) or (i==dim-1 and j==dim-1) or (i==0 and j==dim-1):
        return 3
    elif i==0 or j==0 or i==dim-1 or j==dim-1:
        return 5
    return 8

def find_path(parent, dim, si, sj): #used to find the path from the parent data structure
    i,j = dim-1, dim-1
    path = [(dim-1, dim-1)]
    while (i, j) != (si, sj):
        path.insert(0, parent[i][j])
        (i, j) = parent[i][j]
    return(path)

def inference(dis, N, C, B, E, H, visited, con, dim):
    total = 0
    flag = 1
    while flag != 0:
        flag = 0
        for i in range(dim):
            for j in range(dim):
                if visited[i][j]==1 and dis[i][j]!=1:
                    if H[i][j]==0:
                        continue
                    if C[i][j]==B[i][j]:
                        for I in [i-1,i,i+1]:
                            for J in [j-1,j,j+1]:
                                if (I,J) == (i,j) or I>=dim or J>=dim or I<0 or J<0:
                                    continue
                                if con[I][J]==-1:
                                    con[I][J] = 0
                                    for L in [I-1,I,I+1]:
                                        for M in [J-1,J,J+1]:
                                            if (L,M)==(I,J) or L>=dim or M>=dim or L<0 or M<0:
                                                continue
                                            E[L][M] = E[L][M] + 1
                                            H[L][M] = H[L][M] - 1
                                    flag = flag + 1
                    if N[i][j] - C[i][j] == E[i][j]:
                        for I in [i-1,i,i+1]:
                            for J in [j-1,j,j+1]:
                                if (I,J) == (i,j) or I>=dim or J>=dim or I<0 or J<0:
                                    continue
                                if con[I][J]==-1:
                                    con[I][J] = 1
                                    dis[I][J] = 1
                                    for L in [I-1,I,I+1]:
                                        for M in [J-1,J,J+1]:
                                            if (L,M)==(I,J) or L>=dim or M>=dim or L<0 or M<0:
                                                continue
                                            B[L][M] = B[L][M] + 1
                                            H[L][M] = H[L][M] - 1
                                    flag = flag + 1
        total = total + flag
    return total

def check_path(path, dis):
    for (i, j) in path:
        if dis[i][j] == 1:
            return False
    return True

def con_vis(con, vis, dim):
    c = 0
    for i in range(dim):
        for j in range(dim):
            if con[i][j] != -1:
                c = c + 1
    return c-vis

def agent_3(grid, dim, P, heu):
    
    start = timeit.default_timer() #recording time stamp to measure run time

    dis = [[0 for i in range(dim)] for j in range(dim)] #used to represent the gridworld that has been discovered (list of lists)
    
    N = [[count_neighbors(i,j,dim) for j in range(dim)] for i in range(dim)] #Data Structure to store number of neighbors each cells has

    visited = [[0 for i in range(dim)] for j in range(dim)] #Data Structure to store whether or not cell has been visited
    visited[0][0] = 1
    
    con = [[-1 for i in range(dim)] for j in range(dim)] #Data Structure to store whether or not cell has been confirmed empty(0), blocked(1) or unconfirmed(-1)
    con[0][0] = 0
    con[dim-1][dim-1] = 0
    
    C = [[0 for i in range(dim)] for j in range(dim)] #Data Structure to store number of neighbors of a cell that are sensed to be blocked
    C[0][0] = sense(0, 0, grid, dim)

    B = [[0 for i in range(dim)] for j in range(dim)] #Data Structure to store number of neighbors of a cell that are confirmed to be blocked

    E = [[0 for i in range(dim)] for j in range(dim)] #Data Structure to store number of neighbors of a cell that are confirmed to be empty
    E[0][1]=E[1][0]=E[1][1]=E[dim-1][dim-2]=E[dim-2][dim-2]=E[dim-2][dim-1]=1

    H = [[N[i][j] for j in range(dim)]for i in range(dim)] #Data Structure to store number of neighbors of a cell that are unconfirmed
    H[0][1], H[1][0], H[1][1], H[dim-1][dim-2], H[dim-2][dim-2], H[dim-2][dim-1]= 4,4,7,4,7,4

    result = False
    done = False
    
    si = 0 #Co-ordinates of the start node
    sj = 0
    
    final = [] #Data structure to store final trajectory
    
    vis = 0
    plan_t = 0
    bumps = 0

    c_in = inference(dis, N, C, B, E, H, visited, con, dim)
    
    while done != True:

        ps = timeit.default_timer()
        result, parent =a_star(dim, P, dis, heu, si, sj) #planning stage of repeated A*
        plan_t = plan_t + (timeit.default_timer() - ps)
        
        if result == False: #true if grid not solvable
            break
        
        path = find_path(parent, dim, si, sj)

        flag = True
        for (i, j) in path: #agent traversing the planned path
            vis = vis + 1
            if visited[i][j] == 1:
                final.append((i,j))
                continue

            visited[i][j] = 1
            if grid[i][j] == 1: #only updating the grid knowledge after agent bumps into a blocked node
                bumps = bumps + 1
                if con[i][j]==-1:
                    dis[i][j] = 1
                    con[i][j] = 1
                    for I in [i-1,i,i+1]:
                        for J in [j-1,j,j+1]:
                            if (I,J) == (i,j) or I>=dim or J>=dim or I<0 or J<0:
                                continue
                            B[I][J] = B[I][J] + 1
                            H[I][J] = H[I][J] - 1
                (si, sj) = parent[i][j]
                final.pop(len(final)-1)
                flag = False
                c_in = inference(dis, N, C, B, E, H, visited, con, dim)
                break
            else:
                C[i][j] = sense(i, j, grid, dim)
                if con[i][j]==-1:
                    con[i][j] = 0
                    for I in [i-1,i,i+1]:
                        for J in [j-1,j,j+1]:
                            if (I,J) == (i,j) or I>=dim or J>=dim or I<0 or J<0:
                                continue
                            E[I][J] = E[I][J] + 1
                            H[I][J] = H[I][J] - 1
            c_in = inference(dis, N, C, B, E, H, visited, con, dim)
            if c_in != 0:
                if not check_path(path, dis):
                    flag=False
                    (si, sj) = (i,j)
                    break
            final.append((i, j))

        if flag:
            done = True
    
    stop = timeit.default_timer()

    return(result, final, dis, vis, start, stop, plan_t, bumps, con_vis(con, vis, dim)) #recording time stamp to measure run time
